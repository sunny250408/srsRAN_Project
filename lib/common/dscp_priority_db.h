// dscp_priority_db.h - DSCP 기반 RNTI 우선순위 매핑용 글로벌 DB
// lib/common/dscp_priority_db.h
#pragma once
#include <unordered_map>
#include "srsran/ran/rnti.h"

namespace srsran {
inline std::unordered_map<rnti_t, uint8_t> dscp_priority_map;
}




