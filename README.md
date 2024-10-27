# tec-Radar-Scanner


### normal radar

### passive radar

### wifi radar

To create a system where two ESP32 modules collect Channel State Information (CSI), communicate with each other via GPIO, and output an ASCII map of object locations over a 4800 baud serial connection, we need the following setup:

1. **CSI Collection**: Each ESP32 module collects CSI data and shares findings about object locations.
2. **Inter-ESP32 Communication**: One module will act as the **master** and the other as the **slave**, sharing location data via GPIO.
3. **Serial ASCII Map Output**: The master module processes the data and outputs an ASCII map over a 4800 baud serial connection.

### ESP32 Master Code (Module 1)

The **master** ESP32 module will:
- Collect CSI data.
- Communicate with the **slave** ESP32 via GPIO.
- Generate and output an ASCII map.

```c
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_BAUD_RATE 4800
#define UART_TX_PIN 1    // TX pin for serial output
#define UART_RX_PIN 3    // RX pin (not used)
#define GPIO_COMM_PIN 4  // GPIO pin for communication with slave

#define GRID_SIZE 8

// Initialize the ASCII map grid
char grid[GRID_SIZE][GRID_SIZE];

// UART initialization
void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
}

// CSI callback function to process and send data
void csi_callback(void *ctx, wifi_csi_info_t *csi_info) {
    // Example logic to detect position (simplified)
    int x = rand() % GRID_SIZE;
    int y = rand() % GRID_SIZE;

    // Mark object on the grid (for demonstration, we randomize)
    grid[x][y] = 'O';  // 'O' represents an object in the map

    // Send object position to the slave module via GPIO
    gpio_set_level(GPIO_COMM_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_COMM_PIN, 0);
}

// Initialize Wi-Fi and configure CSI
void wifi_init() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = true
    };
    esp_wifi_set_csi_config(&csi_config);
    esp_wifi_set_csi_rx_cb(csi_callback, NULL);
}

// Send ASCII map via UART
void send_ascii_map() {
    uart_write_bytes(UART_NUM_1, "\n8x8 ASCII Map\n", 14);
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            char cell = (grid[i][j] == 'O') ? 'O' : '.';
            uart_write_bytes(UART_NUM_1, &cell, 1);
            uart_write_bytes(UART_NUM_1, " ", 1);
        }
        uart_write_bytes(UART_NUM_1, "\n", 1);
    }
}

// Task to continuously output the ASCII map
void ascii_map_task(void *pvParameter) {
    while (1) {
        send_ascii_map();
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Update every 2 seconds
    }
}

void app_main() {
    uart_init();
    wifi_init();
    gpio_set_direction(GPIO_COMM_PIN, GPIO_MODE_OUTPUT);

    // Initialize the grid with dots
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            grid[i][j] = '.';
        }
    }

    // Create the ASCII map output task
    xTaskCreate(&ascii_map_task, "ascii_map_task", 4096, NULL, 5, NULL);

    // CSI data collection will run in the background
}
```

### ESP32 Slave Code (Module 2)

The **slave** ESP32 module listens for GPIO signals from the master module to update its own grid based on detected CSI data.

```c
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_COMM_PIN 4   // Same GPIO pin for communication

#define GRID_SIZE 8

char grid[GRID_SIZE][GRID_SIZE];

// Initialize grid with dots
void initialize_grid() {
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            grid[i][j] = '.';
        }
    }
}

// GPIO task to detect signals from the master
void gpio_task(void *pvParameter) {
    while (1) {
        int level = gpio_get_level(GPIO_COMM_PIN);
        if (level == 1) {
            int x = rand() % GRID_SIZE;
            int y = rand() % GRID_SIZE;
            grid[x][y] = 'O';  // Update grid cell with detected object
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void app_main() {
    nvs_flash_init();
    gpio_set_direction(GPIO_COMM_PIN, GPIO_MODE_INPUT);
    initialize_grid();

    // Create a GPIO task for monitoring signals from the master
    xTaskCreate(&gpio_task, "gpio_task", 2048, NULL, 5, NULL);
}
```

### Explanation and Workflow

1. **Master Module (ESP32)**:
   - Initializes CSI collection.
   - Detects object positions (simulated in this example) and marks the positions on a grid.
   - Sends data to the slave ESP32 through a GPIO pulse and prints the 8x8 ASCII map every 2 seconds over the 4800 baud serial connection.

2. **Slave Module (ESP32)**:
   - Listens on the designated GPIO pin for pulses from the master.
   - Upon receiving a pulse, it marks an object’s position on its local grid.
   - This setup could be expanded to process signals or combine data for more accurate object location.
