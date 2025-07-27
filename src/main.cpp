#include "FreeRTOS.h"
#include "task.h"
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "qmc_sensor/qmc5883l.h"
#include <cmath>

extern "C" void led_task(void* pvParameters)
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(200);
        printf("LED ON\n");
        gpio_put(LED_PIN, 0);
        vTaskDelay(1000);
        printf("LED OFF\n");
    }
}

extern "C" void compass_task(void* pvParameters)
{
    // Retrieve the pointer to your compass instance:
    auto* compass  = static_cast<QMC5883L*>(pvParameters);

    int loop_count       = 0;
    int successful_reads = 0;
    int failed_reads     = 0;
    int no_data_count    = 0;

    // Initialization
    if(!compass->initialize()) {
        printf("ERROR: couldn't initialize the sensor");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Configuration
    compass->configure();
    vTaskDelay(pdMS_TO_TICKS(1000));
    for(;;) {
        loop_count++;
        printf("Loop #%d\n", loop_count);
        if (compass->isDataReady()) {
            printf("Data ready\n");
            QMC5883L::MagnetometerData data;
            if (compass->readData(data)) {
                successful_reads++;
                printf("SUCCESS\n");
                printf("  Raw: X=%d Y=%d Z=%d\n", data.x_raw, data.y_raw, data.z_raw);
                printf("  Gauss: X=%.3f Y=%.3f Z=%.3f\n",
                       data.x_gauss, data.y_gauss, data.z_gauss);
                printf("  Heading: %.2f°  Temp: %.1f°C\n",
                       data.heading_degrees, data.temperature_celsius);

                float field = sqrtf(
                    data.x_gauss*data.x_gauss +
                    data.y_gauss*data.y_gauss +
                    data.z_gauss*data.z_gauss
                );
                printf("  Field strength: %.3f G\n", field);
            }
            else {
                failed_reads++;
                printf("Failed to read\n");
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else {
            no_data_count++;
            printf("No data\n");
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        // Stats every 20 loops
        if ((loop_count % 20) == 0) {
            printf("\n--- After %d loops ---\n", loop_count);
            printf("Success: %d  Fail: %d  NoData: %d\n",
                   successful_reads, failed_reads, no_data_count);
            printf("Success rate: %.1f%%\n",
                   (successful_reads*100.0f)/loop_count);
            printf("Data-ready rate: %.1f%%\n\n",
                   ((successful_reads+failed_reads)*100.0f)/loop_count);
        }
    }

}


int main()
{
    stdio_init_all();
    static QMC5883L compass(i2c0, 4, 5, 400000, 0x0D);

    if (xTaskCreate(
            compass_task,
            "CompassTask",
            1024,
            &compass,
            2,
            nullptr
        ) != pdPASS)
        {
            printf("ERROR: could not create compass task\n");
        }
    
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1){};
    return 0;
}