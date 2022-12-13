#pragma once

#include <map>
#include <memory>
#include <string>

#include "prometheus/family.h"
#include "prometheus/gauge.h"
#include "prometheus/registry.h"

extern std::shared_ptr<prometheus::Registry> g_pRegistry;

void DimensionAdd(const std::map<std::string, std::string>& dimensions,
                  const std::string& attributeName, int value);