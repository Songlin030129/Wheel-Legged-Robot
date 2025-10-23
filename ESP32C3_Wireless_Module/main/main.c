#include "common_inc.h"

void app_main(void) {
  Main();
  while (1) {
    vTaskDelay(1000);
  }
}