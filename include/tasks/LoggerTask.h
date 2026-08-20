#pragma once

// センサー値とタスク監視情報をUSBシリアルへ出力するFreeRTOSタスク。

void LoggerTask(void *pvParameters);
