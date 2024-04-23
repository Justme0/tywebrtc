#include "src/monitor/monitor.h"

std::shared_ptr<prometheus::Registry> g_pRegistry;

void DimensionAdd(const std::map<std::string, std::string>& dimensions,
                  const std::string& attributeName, int value) {
  prometheus::BuildGauge()
      .Name(attributeName.data())
      .Help(attributeName.data())
      .Register(*g_pRegistry)
      .Add(dimensions)
      .Increment(value);
}

// have bug
// attribute =>
std::map<std::string, prometheus::Family<prometheus::Gauge>*> g_mSet;

void DimensionSet(const std::map<std::string, std::string>& dimensions,
                  const std::string& attributeName, int value) {
  prometheus::Family<prometheus::Gauge>* family = g_mSet[attributeName];
  if (nullptr != family) {
    g_pRegistry->Remove(*family);
  }

  // save it for delete next time
  family = &prometheus::BuildGauge()
                .Name(attributeName.data())
                .Help(attributeName.data())
                .Register(*g_pRegistry);

  family->Add(dimensions).Set(value);
}