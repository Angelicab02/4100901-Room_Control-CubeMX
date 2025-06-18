#include "keypad_driver.h"

static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

void keypad_init(keypad_handle_t* keypad)
{
    // Configura filas como salidas y las pone en bajo
    for (int fila = 0; fila < KEYPAD_ROWS; fila++) {
        GPIO_InitTypeDef cfg = {0};
        cfg.Pin = keypad->row_pins[fila];
        cfg.Mode = GPIO_MODE_OUTPUT_PP;
        cfg.Pull = GPIO_NOPULL;
        cfg.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(keypad->row_ports[fila], &cfg);
        HAL_GPIO_WritePin(keypad->row_ports[fila], keypad->row_pins[fila], GPIO_PIN_RESET);
    }

    // Configura columnas como entradas con pull-up y con interrupción
    for (int col = 0; col < KEYPAD_COLS; col++) {
        GPIO_InitTypeDef cfg = {0};
        cfg.Pin = keypad->col_pins[col];
        cfg.Mode = GPIO_MODE_IT_FALLING;
        cfg.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(keypad->col_ports[col], &cfg);
    }
}

char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin)
{
    char tecla = '\0';
    static uint32_t ultimo_tiempo = 0;
    if (HAL_GetTick() - ultimo_tiempo < 100) {
        return tecla; // Antirrebote
    }
    ultimo_tiempo = HAL_GetTick();

    // Buscar el índice de la columna que generó la interrupción
    int col_idx = -1;
    for (int c = 0; c < KEYPAD_COLS; c++) {
        if (col_pin == keypad->col_pins[c]) {
            col_idx = c;
            break;
        }
    }
    if (col_idx < 0) {
        keypad_init(keypad);
        return '\0';
    }

    // Escanear filas para encontrar la tecla presionada
    for (int fila = 0; fila < KEYPAD_ROWS; fila++) {
        // Poner todas las filas en alto antes de activar una
        for (int f = 0; f < KEYPAD_ROWS; f++)
            HAL_GPIO_WritePin(keypad->row_ports[f], keypad->row_pins[f], GPIO_PIN_SET);

        // Activar la fila actual (poner en bajo)
        HAL_GPIO_WritePin(keypad->row_ports[fila], keypad->row_pins[fila], GPIO_PIN_RESET);

        // Pequeño retardo para estabilizar
        for (volatile int d = 0; d < 1000; d++);

        // Leer la columna correspondiente
        if (HAL_GPIO_ReadPin(keypad->col_ports[col_idx], keypad->col_pins[col_idx]) == GPIO_PIN_RESET) {
            tecla = keypad_map[fila][col_idx];
            // Esperar a que se suelte la tecla
            while (HAL_GPIO_ReadPin(keypad->col_ports[col_idx], keypad->col_pins[col_idx]) == GPIO_PIN_RESET);
            break;
        }
    }

    // Restaurar la configuración del keypad para futuras interrupciones
    keypad_init(keypad);
    return tecla;
}