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

#pragma once

#include "ngap_asn1_utils.h"
#include "srsran/asn1/ngap/ngap_ies.h"
#include "srsran/cu_cp/cu_cp_types.h"
#include "srsran/ngap/ngap_handover.h"
#include "srsran/ran/bcd_helper.h"
#include "srsran/ran/cause/ngap_cause.h"
#include "srsran/ran/cu_types.h"
#include "srsran/ran/rb_id.h"
#include "srsran/ran/up_transport_layer_info.h"
#include "srsran/srslog/srslog.h"
#include <variant>

namespace srsran {
namespace srs_cu_cp {

// Helper to create PDU from NGAP message.
template <class T>
byte_buffer pack_into_pdu(const T& msg, const char* context_name = nullptr)
{
  context_name = context_name == nullptr ? __FUNCTION__ : context_name;
  byte_buffer   pdu{};
  asn1::bit_ref bref{pdu};
  if (msg.pack(bref) == asn1::SRSASN_ERROR_ENCODE_FAIL) {
    srslog::fetch_basic_logger("NGAP").error("Failed to pack message in {} - discarding it", context_name);
    pdu.clear();
  }
  return pdu;
}

/// \brief  Convert CU-CP security result to NGAP security result.
/// \param security_result The CU-CP security result.
/// \return The NGAP security result.
inline asn1::ngap::security_result_s cu_cp_security_result_to_ngap_security_result(security_result_t security_result)
{
  asn1::ngap::security_result_s ngap_security_result;

  if (security_result.confidentiality_protection_result == confidentiality_protection_result_t::performed) {
    ngap_security_result.confidentiality_protection_result =
        asn1::ngap::confidentiality_protection_result_opts::options::performed;
  } else {
    ngap_security_result.confidentiality_protection_result =
        asn1::ngap::confidentiality_protection_result_opts::options::not_performed;
  }

  if (security_result.integrity_protection_result == integrity_protection_result_t::performed) {
    ngap_security_result.integrity_protection_result.value =
        asn1::ngap::integrity_protection_result_opts::options::performed;
  } else {
    ngap_security_result.integrity_protection_result =
        asn1::ngap::integrity_protection_result_opts::options::not_performed;
  }

  return ngap_security_result;
}

/// \brief Convert CU-CP Associated QoS Flow to NGAP Associated QoS Flow Item.
/// \param cu_cp_qos_flow The CU-CP Associated QoS Flow.
/// \return The NGAP Associated QoS Flow Item.
inline asn1::ngap::associated_qos_flow_item_s
cu_cp_assoc_qos_flow_to_ngap_assoc_qos_flow_item(cu_cp_associated_qos_flow cu_cp_qos_flow)
{
  asn1::ngap::associated_qos_flow_item_s asn1_assoc_qos_item;

  asn1_assoc_qos_item.qos_flow_id = qos_flow_id_to_uint(cu_cp_qos_flow.qos_flow_id);

  if (cu_cp_qos_flow.qos_flow_map_ind.has_value()) {
    asn1_assoc_qos_item.qos_flow_map_ind_present = true;

    if (cu_cp_qos_flow.qos_flow_map_ind.value() == cu_cp_qos_flow_map_ind::ul) {
      asn1_assoc_qos_item.qos_flow_map_ind.value =
          asn1::ngap::associated_qos_flow_item_s::qos_flow_map_ind_opts::options::ul;
    } else {
      asn1_assoc_qos_item.qos_flow_map_ind.value =
          asn1::ngap::associated_qos_flow_item_s::qos_flow_map_ind_opts::options::dl;
    }
  }

  return asn1_assoc_qos_item;
}

/// \brief Convert CU-CP QoS Flow Per TNL Info to NGAP QoS Flow Per TNL Info.
/// \param cu_cp_qos_flow_info The CU-CP QoS Flow Per TNL Info.
/// \return The NGAP QoS Flow Per TNL Info.
inline asn1::ngap::qos_flow_per_tnl_info_s
cu_cp_qos_flow_per_tnl_info_to_ngap_qos_flow_per_tnl_info(const cu_cp_qos_flow_per_tnl_information& cu_cp_qos_flow_info)
{
  asn1::ngap::qos_flow_per_tnl_info_s ngap_qos_flow_info;

  up_transport_layer_info_to_asn1(ngap_qos_flow_info.up_transport_layer_info, cu_cp_qos_flow_info.up_tp_layer_info);

  for (const auto& cu_cp_assoc_qos_item : cu_cp_qos_flow_info.associated_qos_flow_list) {
    asn1::ngap::associated_qos_flow_item_s ngap_assoc_qos_item =
        cu_cp_assoc_qos_flow_to_ngap_assoc_qos_flow_item(cu_cp_assoc_qos_item);

    ngap_qos_flow_info.associated_qos_flow_list.push_back(ngap_assoc_qos_item);
  }

  return ngap_qos_flow_info;
}

/// \brief Convert \c ngap_cause_t type to NGAP cause.
/// \param cause The ngap_cause_t type.
/// \return The NGAP cause.
inline asn1::ngap::cause_c cause_to_asn1(ngap_cause_t cause)
{
  asn1::ngap::cause_c asn1_cause;

  if (const auto* result = std::get_if<ngap_cause_radio_network_t>(&cause)) {
    asn1_cause.set_radio_network() = static_cast<asn1::ngap::cause_radio_network_opts::options>(*result);
    return asn1_cause;
  }
  if (const auto* result = std::get_if<ngap_cause_transport_t>(&cause)) {
    asn1_cause.set_transport() = static_cast<asn1::ngap::cause_transport_opts::options>(*result);
    return asn1_cause;
  }
  if (const auto* result = std::get_if<cause_nas_t>(&cause)) {
    asn1_cause.set_nas() = static_cast<asn1::ngap::cause_nas_opts::options>(*result);
    return asn1_cause;
  }
  if (const auto* result = std::get_if<cause_protocol_t>(&cause)) {
    asn1_cause.set_protocol() = static_cast<asn1::ngap::cause_protocol_opts::options>(*result);
    return asn1_cause;
  }
  if (const auto* result = std::get_if<ngap_cause_misc_t>(&cause)) {
    asn1_cause.set_misc() = static_cast<asn1::ngap::cause_misc_opts::options>(*result);
    return asn1_cause;
  }

  report_fatal_error("Cannot convert cause to NGAP type:{}", cause);
  return asn1_cause;
}

/// \brief Convert NGAP ASN1 cause to \c ngap_cause_t type.
/// \param asn1_cause The ASN1 NGAP cause.
/// \return The ngap_cause_t type.
inline ngap_cause_t asn1_to_cause(asn1::ngap::cause_c asn1_cause)
{
  ngap_cause_t cause;

  switch (asn1_cause.type()) {
    case asn1::ngap::cause_c::types_opts::radio_network:
      cause = static_cast<ngap_cause_radio_network_t>(asn1_cause.radio_network().value);
      break;
    case asn1::ngap::cause_c::types_opts::transport:
      cause = static_cast<ngap_cause_transport_t>(asn1_cause.transport().value);
      break;
    case asn1::ngap::cause_c::types_opts::nas:
      cause = static_cast<cause_nas_t>(asn1_cause.nas().value);
      break;
    case asn1::ngap::cause_c::types_opts::protocol:
      cause = static_cast<cause_protocol_t>(asn1_cause.protocol().value);
      break;
    case asn1::ngap::cause_c::types_opts::misc:
      cause = static_cast<ngap_cause_misc_t>(asn1_cause.misc().value);
      break;
    default:
      report_fatal_error("Cannot convert NGAP ASN.1 cause {} to common type", fmt::underlying(asn1_cause.type().value));
  }

  return cause;
}

/// \brief Convert CU-CP QoS Flow Failed to Setup Item to NGAP QoS Flow With Cause Item.
/// \param cu_cp_failed_item The CU-CP QoS Flow Failed to Setup Item.
/// \return The NGAP QoS Flow With Cause Item.
inline asn1::ngap::qos_flow_with_cause_item_s cu_cp_qos_flow_failed_to_setup_item_to_ngap_qos_flow_with_cause_item(
    cu_cp_qos_flow_failed_to_setup_item cu_cp_failed_item)
{
  asn1::ngap::qos_flow_with_cause_item_s asn1_failed_item;
  asn1_failed_item.qos_flow_id = qos_flow_id_to_uint(cu_cp_failed_item.qos_flow_id);
  asn1_failed_item.cause       = cause_to_asn1(cu_cp_failed_item.cause);

  return asn1_failed_item;
}

/// \brief Convert CU-CP NRCGI to NR Cell Identity.
/// \param ngap_cgi The NGAP NRCGI.
/// \return The NR Cell Identity.
inline nr_cell_identity cu_cp_nrcgi_to_nr_cell_identity(asn1::ngap::nr_cgi_s& ngap_cgi)
{
  return nr_cell_identity::create(ngap_cgi.nr_cell_id.to_number()).value();
}

/// \brief Convert CU-CP NRCGI to NR Cell Identity.
/// \param ngap_cgi The NGAP NRCGI.
/// \return The NR Cell Identity.
inline asn1::ngap::user_location_info_nr_s
cu_cp_user_location_info_to_asn1(const cu_cp_user_location_info_nr& cu_cp_user_location_info)
{
  asn1::ngap::user_location_info_nr_s asn1_user_location_info;

  // Fill NR CGI.
  asn1_user_location_info.nr_cgi.nr_cell_id.from_number(cu_cp_user_location_info.nr_cgi.nci.value());
  asn1_user_location_info.nr_cgi.plmn_id = cu_cp_user_location_info.nr_cgi.plmn_id.to_bytes();
  // Fill TAI.
  asn1_user_location_info.tai.plmn_id = cu_cp_user_location_info.tai.plmn_id.to_bytes();
  asn1_user_location_info.tai.tac.from_number(cu_cp_user_location_info.tai.tac);
  // Fill timestamp.
  if (cu_cp_user_location_info.time_stamp.has_value()) {
    asn1_user_location_info.time_stamp_present = true;
    asn1_user_location_info.time_stamp.from_number(cu_cp_user_location_info.time_stamp.value());
  }

  return asn1_user_location_info;
}

/// \brief Convert ASN.1 cause to a human-readable string.
/// \param cause The ASN.1 cause.
/// \return The humand-readable string.
inline std::string asn1_cause_to_string(const asn1::ngap::cause_c& cause)
{
  std::string cause_str;

  switch (cause.type()) {
    case asn1::ngap::cause_c::types_opts::radio_network:
      cause_str = cause.radio_network().to_string();
      break;
    case asn1::ngap::cause_c::types_opts::transport:
      cause_str = cause.transport().to_string();
      break;
    case asn1::ngap::cause_c::types_opts::protocol:
      cause_str = cause.protocol().to_string();
      break;
    case asn1::ngap::cause_c::types_opts::nas:
      cause_str = cause.nas().to_string();
      break;
    case asn1::ngap::cause_c::types_opts::misc:
      cause_str = cause.misc().to_string();
      break;
    default:
      cause_str = "unknown";
      break;
  }

  return cause_str;
}

/// \brief Convert common type Initial Context Setup Response message to NGAP Initial Context Setup Response message.
/// \param[out] resp The ASN1 NGAP Initial Context Setup Response message.
/// \param[in] cu_cp_resp The CU-CP Initial Context Setup Response message.
/// \return True on success, otherwise false.
template <typename template_asn1_item>
inline bool pdu_session_res_setup_response_item_to_asn1(template_asn1_item&                              asn1_resp,
                                                        const cu_cp_pdu_session_res_setup_response_item& resp)
{
  asn1_resp.pdu_session_id = pdu_session_id_to_uint(resp.pdu_session_id);

  asn1::ngap::pdu_session_res_setup_resp_transfer_s response_transfer;

  // Fill DL QoS flow per TNL information.
  response_transfer.dl_qos_flow_per_tnl_info = cu_cp_qos_flow_per_tnl_info_to_ngap_qos_flow_per_tnl_info(
      resp.pdu_session_resource_setup_response_transfer.dlqos_flow_per_tnl_info);

  // Fill additional DL QoS flow per TNL information.
  for (const auto& cu_cp_qos_flow_info :
       resp.pdu_session_resource_setup_response_transfer.add_dl_qos_flow_per_tnl_info) {
    asn1::ngap::qos_flow_per_tnl_info_item_s ngap_qos_flow_info;
    ngap_qos_flow_info.qos_flow_per_tnl_info =
        cu_cp_qos_flow_per_tnl_info_to_ngap_qos_flow_per_tnl_info(cu_cp_qos_flow_info);
    response_transfer.add_dl_qos_flow_per_tnl_info.push_back(ngap_qos_flow_info);
  }

  // Fill QoS flow failed to setup list.
  for (const auto& failed_item : resp.pdu_session_resource_setup_response_transfer.qos_flow_failed_to_setup_list) {
    asn1::ngap::qos_flow_with_cause_item_s asn1_failed_item =
        cu_cp_qos_flow_failed_to_setup_item_to_ngap_qos_flow_with_cause_item(failed_item);
    response_transfer.qos_flow_failed_to_setup_list.push_back(asn1_failed_item);
  }

  // Fill security result.
  if (resp.pdu_session_resource_setup_response_transfer.security_result.has_value()) {
    response_transfer.security_result_present = true;
    response_transfer.security_result         = cu_cp_security_result_to_ngap_security_result(
        resp.pdu_session_resource_setup_response_transfer.security_result.value());
  }

  // Pack pdu_session_res_setup_resp_transfer_s.
  byte_buffer pdu = pack_into_pdu(response_transfer);

  if (!asn1_resp.pdu_session_res_setup_resp_transfer.resize(pdu.length())) {
    return false;
  }
  std::copy(pdu.begin(), pdu.end(), asn1_resp.pdu_session_res_setup_resp_transfer.begin());

  return true;
}

/// \brief Convert common type modify response item to ASN1 type message.
/// \param[out] asn1_resp The ASN1 NGAP struct.
/// \param[in] resp The common type struct.
/// \return True on success, otherwise false.
template <typename template_asn1_item>
inline bool pdu_session_res_modify_response_item_to_asn1(template_asn1_item& asn1_resp,
                                                         const cu_cp_pdu_session_resource_modify_response_item& resp)
{
  asn1_resp.pdu_session_id = pdu_session_id_to_uint(resp.pdu_session_id);

  asn1::ngap::pdu_session_res_modify_resp_transfer_s response_transfer;

  // Fill additional DL QoS f low per TNL information.
  for (const auto& cu_cp_qos_flow_info : resp.transfer.add_dl_qos_flow_per_tnl_info) {
    asn1::ngap::qos_flow_per_tnl_info_item_s ngap_qos_flow_info;
    ngap_qos_flow_info.qos_flow_per_tnl_info =
        cu_cp_qos_flow_per_tnl_info_to_ngap_qos_flow_per_tnl_info(cu_cp_qos_flow_info);
    response_transfer.add_dl_qos_flow_per_tnl_info.push_back(ngap_qos_flow_info);
  }

  if (resp.transfer.qos_flow_add_or_modify_response_list.has_value()) {
    for (const auto& qos_flow : resp.transfer.qos_flow_add_or_modify_response_list.value()) {
      asn1::ngap::qos_flow_add_or_modify_resp_item_s asn1_item;
      asn1_item.qos_flow_id = qos_flow_id_to_uint(qos_flow.qos_flow_id);
      response_transfer.qos_flow_add_or_modify_resp_list.push_back(asn1_item);
    }
  }

  // Pack pdu_session_res_modify_resp_transfer_s.
  byte_buffer pdu = pack_into_pdu(response_transfer);

  if (!asn1_resp.pdu_session_res_modify_resp_transfer.resize(pdu.length())) {
    return false;
  }
  std::copy(pdu.begin(), pdu.end(), asn1_resp.pdu_session_res_modify_resp_transfer.begin());

  return true;
}

/// \brief Convert common type modify response item to ASN1 type message.
/// \param[out] asn1_resp The ASN1 NGAP struct.
/// \param[in] resp The common type struct.
/// \return True on success, otherwise false.
template <typename template_asn1_item>
inline bool pdu_session_res_failed_to_modify_item_to_asn1(template_asn1_item& asn1_resp,
                                                          const cu_cp_pdu_session_resource_failed_to_modify_item& resp)
{
  asn1_resp.pdu_session_id = pdu_session_id_to_uint(resp.pdu_session_id);

  asn1::ngap::pdu_session_res_modify_unsuccessful_transfer_s response_transfer;
  response_transfer.cause = cause_to_asn1(resp.unsuccessful_transfer.cause);

  // Pack transfer.
  byte_buffer pdu = pack_into_pdu(response_transfer);

  if (!asn1_resp.pdu_session_res_modify_unsuccessful_transfer.resize(pdu.length())) {
    return false;
  }
  std::copy(pdu.begin(), pdu.end(), asn1_resp.pdu_session_res_modify_unsuccessful_transfer.begin());

  return true;
}

/// \brief Convert common type Initial Context Setup Response message to NGAP Initial Context Setup Response message.
/// \param[out] resp The ASN1 NGAP Initial Context Setup Response message.
/// \param[in] cu_cp_resp The CU-CP Initial Context Setup Response message.
/// \return True on success, otherwise false.
template <typename template_asn1_item>
inline bool pdu_session_res_setup_failed_item_to_asn1(template_asn1_item&                            asn1_resp,
                                                      const cu_cp_pdu_session_res_setup_failed_item& resp)
{
  asn1_resp.pdu_session_id = pdu_session_id_to_uint(resp.pdu_session_id);

  asn1::ngap::pdu_session_res_setup_unsuccessful_transfer_s setup_unsuccessful_transfer;
  setup_unsuccessful_transfer.cause = cause_to_asn1(resp.unsuccessful_transfer.cause);

  // TODO: Add crit diagnostics.

  // Pack pdu_session_res_setup_unsuccessful_transfer_s.
  byte_buffer pdu = pack_into_pdu(setup_unsuccessful_transfer);

  if (!asn1_resp.pdu_session_res_setup_unsuccessful_transfer.resize(pdu.length())) {
    return false;
  }
  std::copy(pdu.begin(), pdu.end(), asn1_resp.pdu_session_res_setup_unsuccessful_transfer.begin());

  return true;
}

/// \brief Convert ASN.1 GUAMI to a common type.
/// \param asn1_guami The ASN.1 GUAMI.
/// \return The common type GUAMI.
inline guami_t asn1_to_guami(const asn1::ngap::guami_s& asn1_guami)
{
  guami_t guami;
  guami.plmn          = plmn_identity::from_bytes(asn1_guami.plmn_id.to_bytes()).value();
  guami.amf_region_id = asn1_guami.amf_region_id.to_number();
  guami.amf_set_id    = asn1_guami.amf_set_id.to_number();
  guami.amf_pointer   = asn1_guami.amf_pointer.to_number();

  return guami;
}

/// \brief Converts type \c security_indication to an ASN.1 type.
/// \param asn1obj ASN.1 object where the result of the conversion is stored.
/// \param security_indication Security Indication IE contents.
inline void security_indication_to_asn1(asn1::ngap::security_ind_s& asn1obj, const security_indication_t& security_ind)
{
  switch (security_ind.integrity_protection_ind) {
    case integrity_protection_indication_t::not_needed:
    case integrity_protection_indication_t::preferred:
    case integrity_protection_indication_t::required:
      asn1obj.integrity_protection_ind.value =
          static_cast<asn1::ngap::integrity_protection_ind_opts::options>(security_ind.integrity_protection_ind);
      break;
    default:
      report_fatal_error("Cannot convert security indication to NGAP type");
  }

  switch (security_ind.confidentiality_protection_ind) {
    case confidentiality_protection_indication_t::not_needed:
    case confidentiality_protection_indication_t::preferred:
    case confidentiality_protection_indication_t::required:
      asn1obj.confidentiality_protection_ind.value =
          static_cast<asn1::ngap::confidentiality_protection_ind_opts::options>(security_ind.integrity_protection_ind);
      break;
    default:
      report_fatal_error("Cannot convert security indication to NGAP type");
  }
}

/// \brief Converts type \c security_indication to an ASN.1 type.
/// \param asn1obj ASN.1 object where the result of the conversion is stored.
/// \param security_indication Security Indication IE contents.
inline void asn1_to_security_indication(security_indication_t& security_ind, const asn1::ngap::security_ind_s& asn1obj)
{
  switch (asn1obj.integrity_protection_ind) {
    case asn1::ngap::integrity_protection_ind_opts::not_needed:
    case asn1::ngap::integrity_protection_ind_opts::preferred:
    case asn1::ngap::integrity_protection_ind_opts::required:
      security_ind.integrity_protection_ind =
          static_cast<integrity_protection_indication_t>(asn1obj.integrity_protection_ind.value);
      break;
    default:
      srslog::fetch_basic_logger("NGAP").error("Cannot convert security indication to NGAP type");
  }

  switch (asn1obj.confidentiality_protection_ind) {
    case asn1::ngap::confidentiality_protection_ind_opts::not_needed:
    case asn1::ngap::confidentiality_protection_ind_opts::preferred:
    case asn1::ngap::confidentiality_protection_ind_opts::required:
      security_ind.confidentiality_protection_ind =
          static_cast<confidentiality_protection_indication_t>(asn1obj.confidentiality_protection_ind.value);
      break;
    default:
      srslog::fetch_basic_logger("NGAP").error("Cannot convert security indication to NGAP type");
  }
}

inline void asn1_to_handov_type(ngap_handov_type& handov_type, const asn1::ngap::handov_type_e& asn1_handov_type)
{
  switch (asn1_handov_type) {
    case asn1::ngap::handov_type_opts::options::intra5gs:
      handov_type = ngap_handov_type::intra5gs;
      break;
    case asn1::ngap::handov_type_opts::options::fivegs_to_eps:
      handov_type = ngap_handov_type::fivegs_to_eps;
      break;
    case asn1::ngap::handov_type_opts::options::eps_to_5gs:
      handov_type = ngap_handov_type::eps_to_5gs;
      break;
    case asn1::ngap::handov_type_opts::options::fivegs_to_utran:
      handov_type = ngap_handov_type::fivegs_to_utran;
      break;
    default:
      // error
      srslog::fetch_basic_logger("NGAP").error("Cannot convert handov type to NGAP type");
  }
}

inline s_nssai_t ngap_asn1_to_s_nssai(const asn1::ngap::s_nssai_s& asn1_s_nssai)
{
  s_nssai_t s_nssai;
  s_nssai.sst = slice_service_type{(uint8_t)asn1_s_nssai.sst.to_number()};
  if (asn1_s_nssai.sd_present) {
    s_nssai.sd = slice_differentiator::create(asn1_s_nssai.sd.to_number()).value();
  }

  return s_nssai;
}

inline asn1::ngap::s_nssai_s s_nssai_to_asn1(const s_nssai_t& s_nssai)
{
  asn1::ngap::s_nssai_s asn1_s_nssai;
  asn1_s_nssai.sst.from_number(s_nssai.sst.value());

  if (s_nssai.sd.is_set()) {
    asn1_s_nssai.sd_present = true;
    asn1_s_nssai.sd.from_number(s_nssai.sd.value());
  }

  return asn1_s_nssai;
}

inline cu_cp_tai ngap_asn1_to_tai(const asn1::ngap::tai_s& asn1_tai)
{
  cu_cp_tai tai;
  tai.plmn_id = plmn_identity::from_bytes(asn1_tai.plmn_id.to_bytes()).value();
  tai.tac     = asn1_tai.tac.to_number();

  return tai;
}

inline bool asn1_to_security_context(security::security_context&           sec_ctxt,
                                     const asn1::ngap::ue_security_cap_s&  asn1_sec_cap,
                                     const asn1::ngap::security_context_s& asn1_sec_ctxt)
{
  copy_asn1_key(sec_ctxt.k, asn1_sec_ctxt.next_hop_nh);
  sec_ctxt.ncc = asn1_sec_ctxt.next_hop_chaining_count;
  fill_supported_algorithms(sec_ctxt.supported_int_algos, asn1_sec_cap.nr_integrity_protection_algorithms);
  fill_supported_algorithms(sec_ctxt.supported_enc_algos, asn1_sec_cap.nr_encryption_algorithms);
  srslog::fetch_basic_logger("NGAP").debug(asn1_sec_ctxt.next_hop_nh.data(), 32, "K_gnb");
  srslog::fetch_basic_logger("NGAP").debug("Supported integrity algorithms: {}", sec_ctxt.supported_int_algos);
  srslog::fetch_basic_logger("NGAP").debug("Supported ciphering algorithms: {}", sec_ctxt.supported_enc_algos);

  return true;
}

/// \brief Convert NGAP ASN.1 to \c nr_cell_global_id_t.
/// \param[in] asn1_nr_cgi The ASN.1 type nr cgi.
/// \return The common type nr cgi.
inline nr_cell_global_id_t ngap_asn1_to_nr_cgi(const asn1::ngap::nr_cgi_s& asn1_nr_cgi)
{
  nr_cell_global_id_t nr_cgi;

  // Fill NR cell ID.
  nr_cgi.nci = nr_cell_identity::create(asn1_nr_cgi.nr_cell_id.to_number()).value();

  // Fill PLMN ID.
  nr_cgi.plmn_id = plmn_identity::from_bytes(asn1_nr_cgi.plmn_id.to_bytes()).value();

  return nr_cgi;
}

/// \brief Convert \c nr_cell_global_id_t to NGAP ASN.1.
/// \param[in] nr_cgi The common type nr cgi.
/// \return The ASN.1 nr cgi.
inline asn1::ngap::nr_cgi_s nr_cgi_to_ngap_asn1(const nr_cell_global_id_t& nr_cgi)
{
  asn1::ngap::nr_cgi_s asn1_nr_cgi;

  // Fill NR cell ID.
  asn1_nr_cgi.nr_cell_id.from_number(nr_cgi.nci.value());

  // Fill PLMN ID.
  asn1_nr_cgi.plmn_id = nr_cgi.plmn_id.to_bytes();

  return asn1_nr_cgi;
}

inline void asn1_to_source_to_target_transport_container(
    ngap_source_ngran_node_to_target_ngran_node_transparent_container&                container,
    const asn1::ngap::source_ngran_node_to_target_ngran_node_transparent_container_s& asn1_container)
{
  // Fill RRC container.
  container.rrc_container = asn1_container.rrc_container.copy();

  // Fill PDU session res info list.
  for (const auto& asn1_pdu_info_item : asn1_container.pdu_session_res_info_list) {
    ngap_pdu_session_res_info_item pdu_info_item;

    // Fill PDU session ID.
    pdu_info_item.pdu_session_id = uint_to_pdu_session_id(asn1_pdu_info_item.pdu_session_id);

    // Fill QoS flow info list.
    for (const auto& asn1_qos_flow : asn1_pdu_info_item.qos_flow_info_list) {
      ngap_qos_flow_info_item qos_flow;
      // Fill QoS flow ID.
      qos_flow.qos_flow_id = uint_to_qos_flow_id(asn1_qos_flow.qos_flow_id);

      // Fill DL forwarding.
      if (asn1_qos_flow.dl_forwarding_present) {
        qos_flow.dl_forwarding = asn1::enum_to_bool(asn1_qos_flow.dl_forwarding);
      }

      pdu_info_item.qos_flow_info_list.push_back(qos_flow);
    }

    // Fill DRBs to QoS flow map list.
    for (const auto& asn1_drbs_to_qos_flow_map : asn1_pdu_info_item.drbs_to_qos_flows_map_list) {
      ngap_drbs_to_qos_flows_map_item drbs_to_qos_flow_map;

      // Fill DRB ID.
      drbs_to_qos_flow_map.drb_id = uint_to_drb_id(asn1_drbs_to_qos_flow_map.drb_id);

      // Fill Associated QoS flow list.
      for (const auto& asn1_assoc_qos_flow : asn1_drbs_to_qos_flow_map.associated_qos_flow_list) {
        cu_cp_associated_qos_flow assoc_qos_flow;

        // Fill QoS flow ID.
        assoc_qos_flow.qos_flow_id = uint_to_qos_flow_id(asn1_assoc_qos_flow.qos_flow_id);

        // Fill QoS flow map indication.
        if (asn1_assoc_qos_flow.qos_flow_map_ind_present) {
          if (asn1_assoc_qos_flow.qos_flow_map_ind ==
              asn1::ngap::associated_qos_flow_item_s::qos_flow_map_ind_opts::options::ul) {
            assoc_qos_flow.qos_flow_map_ind = cu_cp_qos_flow_map_ind::ul;
          } else if (asn1_assoc_qos_flow.qos_flow_map_ind ==
                     asn1::ngap::associated_qos_flow_item_s::qos_flow_map_ind_opts::options::dl) {
            assoc_qos_flow.qos_flow_map_ind = cu_cp_qos_flow_map_ind::dl;
          }
        }

        drbs_to_qos_flow_map.associated_qos_flow_list.push_back(assoc_qos_flow);
      }

      pdu_info_item.drbs_to_qos_flows_map_list.push_back(drbs_to_qos_flow_map);
    }

    container.pdu_session_res_info_list.push_back(pdu_info_item);
  }

  // Fill ERAB info list.
  for (const auto& asn1_erab_info_item : asn1_container.erab_info_list) {
    ngap_erab_info_item erab_info_item;

    // Fill ERAB ID.
    erab_info_item.erab_id = asn1_erab_info_item.erab_id;

    // Fill DL forwarding.
    if (asn1_erab_info_item.dl_forwarding_present) {
      erab_info_item.dl_forwarding = asn1::enum_to_bool(asn1_erab_info_item.dl_forwarding);
    }

    container.erab_info_list.push_back(erab_info_item);
  }

  // Fill target cell ID.
  container.target_cell_id = ngap_asn1_to_nr_cgi(asn1_container.target_cell_id.nr_cgi());

  // Fill idx to RFSP.
  if (asn1_container.idx_to_rfsp_present) {
    container.idx_to_rfsp = asn1_container.idx_to_rfsp;
  }

  // Fill UE history info.
  for (const auto& asn1_last_item : asn1_container.ue_history_info) {
    ngap_last_visited_cell_item last_item;

    const auto& asn1_cell_info = asn1_last_item.last_visited_cell_info.ngran_cell();

    // Fill last visited cell info.
    // Fill global cell ID.
    last_item.last_visited_cell_info.global_cell_id = ngap_asn1_to_nr_cgi(asn1_cell_info.global_cell_id.nr_cgi());

    // Fill cell type.
    // Fill cell size.
    switch (asn1_cell_info.cell_type.cell_size) {
      case asn1::ngap::cell_size_opts::options::verysmall:
        last_item.last_visited_cell_info.cell_type.cell_size = ngap_cell_size::verysmall;
        break;
      case asn1::ngap::cell_size_opts::options::small:
        last_item.last_visited_cell_info.cell_type.cell_size = ngap_cell_size::small;
        break;
      case asn1::ngap::cell_size_opts::options::medium:
        last_item.last_visited_cell_info.cell_type.cell_size = ngap_cell_size::medium;
        break;
      case asn1::ngap::cell_size_opts::options::large:
        last_item.last_visited_cell_info.cell_type.cell_size = ngap_cell_size::large;
        break;
      default:
        // error
        srslog::fetch_basic_logger("NGAP").error("Cannot convert ASN.1 cell size to NGAP type");
    }

    // Fill time UE stayed in cell.
    last_item.last_visited_cell_info.time_ue_stayed_in_cell = asn1_cell_info.time_ue_stayed_in_cell;

    // Fill time UE stayed in cell enhanced granularity.
    if (asn1_cell_info.time_ue_stayed_in_cell_enhanced_granularity_present) {
      last_item.last_visited_cell_info.time_ue_stayed_in_cell_enhanced_granularity =
          asn1_cell_info.time_ue_stayed_in_cell_enhanced_granularity;
    }

    // Fill HO cause value.
    if (asn1_cell_info.ho_cause_value_present) {
      last_item.last_visited_cell_info.ho_cause_value = asn1_to_cause(asn1_cell_info.ho_cause_value);
    }

    container.ue_history_info.push_back(last_item);
  }
}

/// \brief Convert NGAP ASN.1 to \c cu_cp_global_gnb_id.
/// \param[in] asn1_gnb_id The ASN.1 type global gnb id.
/// \return The common type global gnb id.
inline cu_cp_global_gnb_id ngap_asn1_to_global_gnb_id(const asn1::ngap::global_gnb_id_s& asn1_gnb_id)
{
  cu_cp_global_gnb_id gnb_id;

  // Fill PLMN ID.
  gnb_id.plmn_id = plmn_identity::from_bytes(asn1_gnb_id.plmn_id.to_bytes()).value();

  // Fill gNB ID.
  gnb_id.gnb_id.id         = asn1_gnb_id.gnb_id.gnb_id().to_number();
  gnb_id.gnb_id.bit_length = asn1_gnb_id.gnb_id.gnb_id().length();

  return gnb_id;
}

inline bool pdu_session_res_admitted_item_to_asn1(asn1::ngap::pdu_session_res_admitted_item_s& asn1_admitted_item,
                                                  const ngap_pdu_session_res_admitted_item&    admitted_item)
{
  // Fill PDU session ID.
  asn1_admitted_item.pdu_session_id = pdu_session_id_to_uint(admitted_item.pdu_session_id);

  // Fill HO request ACK transfer.
  asn1::ngap::ho_request_ack_transfer_s asn1_req_ack_transfer;
  // Fill DL NGU UP TNL info.
  up_transport_layer_info_to_asn1(asn1_req_ack_transfer.dl_ngu_up_tnl_info,
                                  admitted_item.ho_request_ack_transfer.dl_ngu_up_tnl_info);

  // Fill DL forwarding UP TNL info.
  if (admitted_item.ho_request_ack_transfer.dl_forwarding_up_tnl_info.has_value()) {
    asn1_req_ack_transfer.dl_forwarding_up_tnl_info_present = true;
    up_transport_layer_info_to_asn1(asn1_req_ack_transfer.dl_forwarding_up_tnl_info,
                                    admitted_item.ho_request_ack_transfer.dl_forwarding_up_tnl_info.value());
  }

  // Fill security result.
  if (admitted_item.ho_request_ack_transfer.security_result.has_value()) {
    asn1_req_ack_transfer.security_result_present = true;
    asn1_req_ack_transfer.security_result =
        cu_cp_security_result_to_ngap_security_result(admitted_item.ho_request_ack_transfer.security_result.value());
  }

  // Fill QoS flow setup resp list.
  for (const auto& qos_flow_item : admitted_item.ho_request_ack_transfer.qos_flow_setup_resp_list) {
    asn1::ngap::qos_flow_item_with_data_forwarding_s asn1_qos_flow_item;

    // Fill QoS flow ID.
    asn1_qos_flow_item.qos_flow_id = qos_flow_id_to_uint(qos_flow_item.qos_flow_id);

    // Fill data forwarding accepted.
    if (qos_flow_item.data_forwarding_accepted.has_value()) {
      asn1_qos_flow_item.data_forwarding_accepted_present = true;
      asn1::bool_to_enum(asn1_qos_flow_item.data_forwarding_accepted, qos_flow_item.data_forwarding_accepted.value());
    }

    asn1_req_ack_transfer.qos_flow_setup_resp_list.push_back(asn1_qos_flow_item);
  }

  // Fill QoS flow failed to setup list.
  for (const auto& qos_flow_failed_item : admitted_item.ho_request_ack_transfer.qos_flow_failed_to_setup_list) {
    asn1::ngap::qos_flow_with_cause_item_s asn1_qos_flow_failed_item =
        cu_cp_qos_flow_failed_to_setup_item_to_ngap_qos_flow_with_cause_item(qos_flow_failed_item);

    asn1_req_ack_transfer.qos_flow_failed_to_setup_list.push_back(asn1_qos_flow_failed_item);
  }

  // Fill data forwarding resp DRB list.
  for (const auto& drb_item : admitted_item.ho_request_ack_transfer.data_forwarding_resp_drb_list) {
    asn1::ngap::data_forwarding_resp_drb_item_s asn1_drb_item;

    // Fill DRB ID.
    asn1_drb_item.drb_id = drb_id_to_uint(drb_item.drb_id);

    // Fill DL forwarding UP TNL info.
    if (drb_item.dl_forwarding_up_tnl_info.has_value()) {
      asn1_drb_item.dl_forwarding_up_tnl_info_present = true;
      up_transport_layer_info_to_asn1(asn1_drb_item.dl_forwarding_up_tnl_info,
                                      drb_item.dl_forwarding_up_tnl_info.value());
    }

    // Fill UL forwarding UP TNL info.
    if (drb_item.ul_forwarding_up_tnl_info.has_value()) {
      asn1_drb_item.ul_forwarding_up_tnl_info_present = true;
      up_transport_layer_info_to_asn1(asn1_drb_item.ul_forwarding_up_tnl_info,
                                      drb_item.ul_forwarding_up_tnl_info.value());
    }

    // Fill data forwarding resp DRB list.
    asn1_req_ack_transfer.data_forwarding_resp_drb_list.push_back(asn1_drb_item);
  }

  // Pack HO request ACK transfer.
  asn1_admitted_item.ho_request_ack_transfer = pack_into_pdu(asn1_req_ack_transfer);
  if (asn1_admitted_item.ho_request_ack_transfer.empty()) {
    srslog::fetch_basic_logger("NGAP").error("Error packing HO Request ACK transfer");
    return false;
  }

  return true;
}

inline bool pdu_session_res_failed_to_setup_item_ho_ack_to_asn1(
    asn1::ngap::pdu_session_res_failed_to_setup_item_ho_ack_s& asn1_failed_item,
    const cu_cp_pdu_session_res_setup_failed_item&             failed_item)
{
  // Fill PDU session ID.
  asn1_failed_item.pdu_session_id = pdu_session_id_to_uint(failed_item.pdu_session_id);

  // Fill HO res alloc unsuccessful transfer.
  asn1::ngap::ho_res_alloc_unsuccessful_transfer_s asn1_ho_res_alloc_unsuccessful_transfer;

  // Fill cause.
  asn1_ho_res_alloc_unsuccessful_transfer.cause = cause_to_asn1(failed_item.unsuccessful_transfer.cause);

  // TODO: Add crit diagnostics

  // Pack HO res alloc unsuccessful transfer.
  asn1_failed_item.ho_res_alloc_unsuccessful_transfer = pack_into_pdu(asn1_ho_res_alloc_unsuccessful_transfer);
  if (asn1_failed_item.ho_res_alloc_unsuccessful_transfer.empty()) {
    srslog::fetch_basic_logger("NGAP").error("Error packing Ho Resource Alloc Unsuccessful transfer");
    return false;
  }

  return true;
}

inline bool target_to_source_transport_container_to_asn1(
    byte_buffer&                                                             asn1_container,
    const ngap_target_ngran_node_to_source_ngran_node_transparent_container& container)
{
  asn1::ngap::target_ngran_node_to_source_ngran_node_transparent_container_s asn1_container_struct;
  asn1_container_struct.rrc_container = container.rrc_container.copy();

  asn1_container = pack_into_pdu(asn1_container_struct);
  if (asn1_container.empty()) {
    srslog::fetch_basic_logger("NGAP").error("Error packing target to source transparent container");
    return false;
  }

  return true;
}

/// \brief Convert NGAP ASN.1 to \c cu_cp_five_g_s_tmsi.
/// \param[in] asn1_ue_id The ASN.1 type ue paging ID.
/// \return The common type cu_cp_five_g_s_tmsi.
inline cu_cp_five_g_s_tmsi ngap_asn1_to_ue_paging_id(const asn1::ngap::ue_paging_id_c& asn1_ue_id)
{
  srsran_assert(asn1_ue_id.type() == asn1::ngap::ue_paging_id_c::types_opts::five_g_s_tmsi,
                "Invalid UE paging ID type");

  return cu_cp_five_g_s_tmsi{asn1_ue_id.five_g_s_tmsi().amf_set_id.to_number(),
                             asn1_ue_id.five_g_s_tmsi().amf_pointer.to_number(),
                             asn1_ue_id.five_g_s_tmsi().five_g_tmsi.to_number()};
}

/// \brief Convert NGAP ASN.1 to \c gbr_qos_flow_information.
/// \param[in] asn1_gbr_qos_info The ASN.1 type gbr qos info.
/// \return The common type gbr qos flow information.
inline gbr_qos_flow_information
ngap_asn1_to_gbr_qos_flow_information(const asn1::ngap::gbr_qos_info_s& asn1_gbr_qos_info)
{
  gbr_qos_flow_information gbr_qos_info;
  gbr_qos_info.max_br_dl = asn1_gbr_qos_info.max_flow_bit_rate_dl;
  gbr_qos_info.max_br_ul = asn1_gbr_qos_info.max_flow_bit_rate_ul;
  gbr_qos_info.gbr_dl    = asn1_gbr_qos_info.guaranteed_flow_bit_rate_dl;
  gbr_qos_info.gbr_ul    = asn1_gbr_qos_info.guaranteed_flow_bit_rate_ul;
  if (asn1_gbr_qos_info.max_packet_loss_rate_dl_present) {
    gbr_qos_info.max_packet_loss_rate_dl = asn1_gbr_qos_info.max_packet_loss_rate_dl;
  }
  if (asn1_gbr_qos_info.max_packet_loss_rate_ul_present) {
    gbr_qos_info.max_packet_loss_rate_ul = asn1_gbr_qos_info.max_packet_loss_rate_ul;
  }

  return gbr_qos_info;
}

/// \brief Convert NGAP ASN.1 to \c pdu_session_type_t.
/// \param[out] pdu_session_type The common type pdu session type.
/// \param[in] ans1_pdu_session_type The ASN.1 type pdu session type.
/// \return True, if the conversion was successful, False otherwise.
inline bool asn1_to_pdu_session_type(pdu_session_type_t&                   pdu_session_type,
                                     const asn1::ngap::pdu_session_type_e& ans1_pdu_session_type)
{
  switch (ans1_pdu_session_type) {
    case asn1::ngap::pdu_session_type_e::pdu_session_type_opts::ipv4:
      pdu_session_type = pdu_session_type_t::ipv4;
      break;
    case asn1::ngap::pdu_session_type_e::pdu_session_type_opts::ipv6:
      pdu_session_type = pdu_session_type_t::ipv6;
      break;
    case asn1::ngap::pdu_session_type_e::pdu_session_type_opts::ipv4v6:
      pdu_session_type = pdu_session_type_t::ipv4v6;
      break;
    case asn1::ngap::pdu_session_type_e::pdu_session_type_opts::ethernet:
      pdu_session_type = pdu_session_type_t::ethernet;
      break;
    default:
      srslog::fetch_basic_logger("NGAP").error("Cannot convert ASN.1 PDU session type to common type");
      return false;
  }
  return true;
}

/// \brief Convert common type \c pdu_session_type_t to NGAP ASN.1.
/// \param[in] pdu_session_type The common type pdu session type.
/// \return The ASN.1 type pdu session type.
inline asn1::ngap::pdu_session_type_e pdu_session_type_to_asn1(const pdu_session_type_t& pdu_session_type)
{
  switch (pdu_session_type) {
    case pdu_session_type_t::ipv4:
      return asn1::ngap::pdu_session_type_e::ipv4;
    case pdu_session_type_t::ipv6:
      return asn1::ngap::pdu_session_type_e::ipv6;
    case pdu_session_type_t::ipv4v6:
      return asn1::ngap::pdu_session_type_e::ipv4v6;
    default:
      return asn1::ngap::pdu_session_type_e::ethernet;
  }
}

} // namespace srs_cu_cp
} // namespace srsran
