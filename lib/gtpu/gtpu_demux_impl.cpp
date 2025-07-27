/*
 *
 * Copyright 2021-2025 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "gtpu_demux_impl.h"
#include "gtpu_pdu.h"
#include "ue_context.h"
#include "srsran/adt/byte_buffer.h"
#include "srsran/support/bit_encoding.h"
#include "dscp_priority_db.h"

#include <sys/socket.h>



namespace srsran {
inline const uint8_t* to_byte_ptr(byte_buffer::const_iterator it) {
  return &*it;
}

gtpu_demux_impl::gtpu_demux_impl(gtpu_demux_cfg_t cfg_, dlt_pcap& gtpu_pcap_) :
  cfg(cfg_), gtpu_pcap(gtpu_pcap_), logger(srslog::fetch_basic_logger("GTPU"))
{
  logger.info("GTP-U demux. {}", cfg);
}

void gtpu_demux_impl::start()
{
  stopped.store(false, std::memory_order_relaxed);
  logger.info("GTP-U demux started. {}", cfg);
}

void gtpu_demux_impl::stop()
{
  stopped.store(true, std::memory_order_relaxed);
}

expected<std::unique_ptr<gtpu_demux_dispatch_queue>>
gtpu_demux_impl::add_tunnel(gtpu_teid_t                                  teid,
                            task_executor&                               tunnel_exec,
                            gtpu_tunnel_common_rx_upper_layer_interface* tunnel)
{
  auto dispacth_fn = [this, teid](span<gtpu_demux_pdu_ctx_t> pdus_span) {
    for (gtpu_demux_pdu_ctx_t& pdu_ctx : pdus_span) {
      handle_pdu_impl(teid, pdu_ctx);
    }
  };
  auto batched_queue =
      std::make_unique<gtpu_demux_dispatch_queue>(cfg.queue_size, tunnel_exec, logger, dispacth_fn, cfg.batch_size);

  std::lock_guard<std::mutex> guard(map_mutex);
  auto                        it = teid_to_tunnel.try_emplace(teid, gtpu_demux_tunnel_ctx_t{*batched_queue, tunnel});
  if (not it.second) {
    logger.error("Tunnel already exists. teid={}", teid);
    return make_unexpected(default_error_t{});
  }

  logger.info("Tunnel added. teid={}", teid);
  return batched_queue;
}

bool gtpu_demux_impl::remove_tunnel(gtpu_teid_t teid)
{
  std::lock_guard<std::mutex> guard(map_mutex);
  auto                        it = teid_to_tunnel.find(teid);
  if (it == teid_to_tunnel.end()) {
    logger.error("Tunnel not found. teid={}", teid);
    return false;
  }

  logger.info("Tunnel removed. teid={}", teid);
  teid_to_tunnel.erase(it);
  return true;
}

void gtpu_demux_impl::apply_test_teid(gtpu_teid_t teid)
{
  std::lock_guard<std::mutex> guard(map_mutex);
  test_teid = teid;
}

void gtpu_demux_impl::handle_pdu(byte_buffer pdu, const sockaddr_storage& src_addr)
{
  if (stopped.load(std::memory_order_relaxed)) {
    return;
  }

  uint32_t read_teid = 0;
  if (not cfg.test_mode) {
    if (not gtpu_read_teid(read_teid, pdu, logger)) {
      logger.error("Failed to read TEID from GTP-U PDU. pdu_len={}", pdu.length());
      return;
    }
  }

  std::lock_guard<std::mutex> guard(map_mutex);

  gtpu_teid_t teid{read_teid};

  if (cfg.test_mode) {
    teid = test_teid;
  }

  auto it = teid_to_tunnel.find(teid);
  if (it == teid_to_tunnel.end()) {
    logger.info("Dropped GTP-U PDU, tunnel not found. teid={}", teid);
    return;
  }
  if (not it->second.batched_queue.try_push(gtpu_demux_pdu_ctx_t{std::move(pdu), src_addr})) {
    if (not cfg.warn_on_drop) {
      logger.info("Dropped GTP-U PDU, queue is full. teid={}", teid);
    } else {
      logger.warning("Dropped GTP-U PDU, queue is full. teid={}", teid);
    }
  }
}

void gtpu_demux_impl::handle_pdu_impl(gtpu_teid_t teid, gtpu_demux_pdu_ctx_t pdu_ctx)
{
  if (stopped.load(std::memory_order_relaxed)) {
    return;
  }
  
  if (gtpu_pcap.is_write_enabled()) {
    auto pdu_copy = pdu_ctx.pdu.deep_copy();
    if (not pdu_copy.has_value()) {
      logger.warning("Unable to deep copy PDU for PCAP writer");
    } else {
      gtpu_pcap.push_pdu(std::move(pdu_copy.value()));
    }
  }

  logger.debug(
      pdu_ctx.pdu.begin(), pdu_ctx.pdu.end(), "Forwarding PDU. pdu_len={} teid={}", pdu_ctx.pdu.length(), teid);

  gtpu_tunnel_common_rx_upper_layer_interface* tunnel = nullptr;
  {
    // Get GTP-U tunnel.
    // We lookup the tunnel again, as the tunnel could have been removed between the time PDU processing was enqueued
    // and the time we actually run the task.
    std::lock_guard<std::mutex> guard(map_mutex);
    auto                        it = teid_to_tunnel.find(teid);
    if (it == teid_to_tunnel.end()) {
      logger.info("Dropped GTP-U PDU, tunnel not found. teid={}", teid);
      return;
    }
    tunnel = it->second.tunnel;
  }
  // Forward entire PDU to the tunnel.
  // As removal happens in the same thread as handling the PDU, we no longer need the lock.
  // ✅ ✨ 여기에 넣어야 함 (tunnel 얻은 뒤, handle_pdu 호출 전)
  // IP 헤더에서 DSCP 추출
  const uint8_t* payload = to_byte_ptr(pdu_ctx.pdu.begin());
  uint8_t dscp = (payload[1] & 0xFC) >> 2; // DSCP는 2~7bit (IPv4)

  // UE 객체로부터 RNTI 얻기 (gNB 입장에서는 이미 UE와 연결된 상태라면 가능해야 함)
if (auto* ue = dynamic_cast<srsran::srs_cu_up::ue_context*>(tunnel)) {
    srsran::srs_cu_up::ue_index_t idx = ue->get_index();
     ue->dscp_priority = dscp;
    logger.info("Scheduler: UE index={} (DSCP={} → priority={})",
                static_cast<uint16_t>(idx), dscp, dscp);
}

  tunnel->handle_pdu(std::move(pdu_ctx.pdu), pdu_ctx.src_addr);
}
}
