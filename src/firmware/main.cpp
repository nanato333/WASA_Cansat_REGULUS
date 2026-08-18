#include <Arduino.h>
#include "CanSatData.h"
#include "tasks/ImuTask.h"
#include "tasks/MagTask.h"
#include "tasks/BaroTask.h"
#include "tasks/GnssTask.h"

void setup()
{
  Serial.begin(115200);
  // 1. Mutexと共有データの初期化
  init_cansat_data();

  // 2. 各メンバーが作成したタスクを生成・起動 (すべてCore 1に割り当て)
  xTaskCreatePinnedToCore(ImuTask, "ImuTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(MagTask, "MagTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(BaroTask, "BaroTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(GnssTask, "GnssTask", 4096, NULL, 2, NULL, 1);

  // 3. あなたの制御用タスク等の起動...
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}