#include "transform_chain.h"

#include "cc_to_pitchbend.h"
#include "transpose.h"

namespace transforms {

int apply(uint8_t *packet, uint16_t len) {
  transpose::apply(packet, len);           // note numbers only; disjoint from below
  return cc_to_pitchbend::apply(packet, len);
}

}  // namespace transforms
