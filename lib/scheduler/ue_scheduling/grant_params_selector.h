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
//주로 DL/UL 전송을 위한 Grant생성과 관련된 추천 값들을 제공
//namespace srsran::sched_helper 내부에 스케줄링 관련 구조체 및 함수들이 정의

#pragma once

#include "../cell/cell_harq_manager.h"
#include "../slicing/slice_ue_repository.h"
#include "../support/sch_pdu_builder.h"
#include "srsran/ran/sch/sch_mcs.h"

namespace srsran {

class ue_cell;
class slice_ue;

namespace sched_helper {
//구조체 설명(dl_sched_context)
/// PDCCH and PDSCH parameters recommended for a DL grant.
struct dl_sched_context {
  /// SearchSpace to use.
  //search_space_id(어떤 구조체나 타입 이름 int같은 거)
  //ss_id (그 타입으로 선언된 변수)
  search_space_id ss_id; 
  /// PDSCH time-domain resource index.
  uint8_t pdsch_td_res_index;
  /// Limits on VRBs for DL grant allocation.
  vrb_interval vrb_lims;
  /// Recommended MCS, considering channel state or, in case of reTx, last HARQ MCS.
  sch_mcs_index recommended_mcs;
  /// Recommended number of layers.
  unsigned recommended_ri;
  /// Expected number of RBs to allocate.
  unsigned expected_nof_rbs;
  rnti_t ue_rnti;
};

/// Retrieve recommended PDCCH and PDSCH parameters for a newTx DL grant.
/// 새 DL전송(newTx)을 위한 스케줄 컨텍스트 생성
std::optional<dl_sched_context> get_newtx_dl_sched_context(const slice_ue& u,
                                                           slot_point      pdcch_slot,
                                                           slot_point      pdsch_slot,
                                                           bool            interleaving_enabled,
                                                           unsigned        pending_bytes);
                                                           //const 상수로 취급하겠다는 c++ 문법 키워드 즉, 읽기만 가능하고 수정은 불가능하다.
                                                           //const slice_ue& u는 함수 안에서 u라는 인자를 통해 전달된 slice_ue 객체를 읽기만 할 수 있고, 수정은 불가능하다는 의미
                                                           //slot_point는 타입(자료형)으로, pdcch_slot과 pdsch_slot은 slot_point 타입의 변수로 선언된 것
                                                           //bool은 자료형으로, interleaving_enabled는 bool 타입의 변수로 선언된 것
                                                           //unsigned는 자료형으로, pending_bytes는 unsigned 타입의 변수로 선언된 것

/// Retrieve recommended PDCCH and PDSCH parameters for a reTx DL grant.
/// 재 전송송 (reTx)을 위한 DL shed_context 생성
std::optional<dl_sched_context> get_retx_dl_sched_context(const slice_ue&               u,
                                                          slot_point                    pdcch_slot,
                                                          slot_point                    pdsch_slot,
                                                          bool                          interleaving_enabled,
                                                          const dl_harq_process_handle& h_dl);

/// Select DL VRBs to allocate for a newTx.
vrb_interval compute_newtx_dl_vrbs(const dl_sched_context& decision_ctxt,
                                   const vrb_bitmap&       used_vrbs,
                                   unsigned                max_nof_rbs = MAX_NOF_PRBS);

/// Select DL VRBs to allocate for a reTx.
vrb_interval compute_retx_dl_vrbs(const dl_sched_context& decision_ctxt, const vrb_bitmap& used_vrbs);

/// PDCCH and PUSCH parameters recommended for a UL grant.
///struct는 서로 관련 있는 여러 데이터를 하나로 묶는 사용자 정의 자료형
struct ul_sched_context {
  /// SearchSpace to use.
  search_space_id ss_id;
  /// PUSCH time-domain resource index.
  uint8_t pusch_td_res_index;
  /// Limits on VRBs for UL grant allocation.
  vrb_interval vrb_lims;
  /// Limits for grant size in RBs.
  interval<unsigned> nof_rb_lims;
  /// Recommended MCS, considering channel state or, in case of reTx, last HARQ MCS.
  sch_mcs_index recommended_mcs;
  /// Expected number of RBs to allocate.
  unsigned expected_nof_rbs;
  /// PUSCH config params.
  pusch_config_params pusch_cfg;
};

/// Retrieve recommended PDCCH and PUSCH parameters for a newTx UL grant.
std::optional<ul_sched_context> get_newtx_ul_sched_context(const slice_ue& u,
                                                           slot_point      pdcch_slot,
                                                           slot_point      pusch_slot,
                                                           unsigned        uci_nof_harq_bits,
                                                           unsigned        pending_bytes);

/// Retrieve recommended PDCCH and PUSCH parameters for a reTx UL grant.
std::optional<ul_sched_context> get_retx_ul_sched_context(const slice_ue&               u,
                                                          slot_point                    pdcch_slot,
                                                          slot_point                    pusch_slot,
                                                          unsigned                      uci_nof_harq_bits,
                                                          const ul_harq_process_handle& h_ul);

/// Select UL VRBs to allocate for a newTx.
vrb_interval compute_newtx_ul_vrbs(const ul_sched_context& decision_ctxt,
                                   const vrb_bitmap&       used_vrbs,
                                   unsigned                max_nof_rbs = MAX_NOF_PRBS);

/// Select UL VRBs to allocate for a reTx.
vrb_interval compute_retx_ul_vrbs(const ul_sched_context& decision_ctxt, const vrb_bitmap& used_vrbs);

} // namespace sched_helper
} // namespace srsran
