#include <stdbool.h>

#include "OTA_Manager.h"

extern "C" const char g_evse_safe_manual_ota_marker[] __attribute__((used)) =
    "EVSE-SAFE-OTA:" CURRENT_VERSION;

extern "C" bool verifyRollbackLater() {
  // Arduino core defaults to validating OTA images during initArduino(),
  // before our app-level health checks run. Returning true keeps the image
  // in pending_verify so OTA_Manager can confirm or reject it deliberately.
  return true;
}
