#pragma once
#include "esphome.h"
#include "esphome/core/component.h"

namespace esphome {
namespace valve {

class Valve : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
};


}  // namespace valve
}  // namespace esphome