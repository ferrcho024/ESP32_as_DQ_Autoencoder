#include <FS.h>
#include <SPIFFS.h>
#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_spiffs.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
//#include "lwip/apps/sntp.h"

#include "connectivity.h"

void initialize_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        printf("SPIFFS mount error (%d)\n", ret);
        return;
    }
}

void create_file(const char *file_path) {
    
    // Intentar abrir el file
    FILE *file = fopen(file_path, "r");
    if (file != NULL) {
        // El file existe, cerrarlo y eliminarlo
        fclose(file);
        if (remove(file_path) == 0) {
            printf("Existing file deleted: %s\n", file_path);
        } else {
            printf("Error deleting the existing file: %s\n", file_path);
        }
    } else {
        printf("File does not exist\n");
    }

    // Crear el file
    file = fopen(file_path, "w");
    if (file == NULL) {
        printf("Error creating the file\n");
        return;
    }

    fclose(file);

}

void write_data_to_file(const char* file_path, float value_df, float value_nova) {
    FILE *file = fopen(file_path, "a");
    if (file == NULL) {
        printf("Error opening the file for writing\n");
        return;
    }

    // Obtener la fecha y hora actual
    struct tm timeinfo = get_current_time();

    fprintf(file, "[%04d-%02d-%02d %02d:%02d:%02d],%.5f,%.5f\n",
                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, value_df, value_nova);

    fclose(file);

    //printf("DF: %.5f, ** NOVA: %.5f\n", value_df, value_nova);

    // Almacenamiento en el file temporal. En este se guarda solo el dato sin fecha para facilidad en la extraccion de los datos para los cálculos
    FILE *file2 = fopen("/spiffs/temp_df.txt", "a");
    if (file2 == NULL) {
        printf("Error opening the file for writing\n");
        return;
    }

    fprintf(file2, "%.5f\n", value_df);

    fclose(file2);

    FILE *file3 = fopen("/spiffs/temp_nova.txt", "a");
    if (file3 == NULL) {
        printf("Error opening the file for writing\n");
        return;
    }

    fprintf(file3, "%.5f\n", value_nova);

    fclose(file3);

}

void write_text_to_file(const char* file_path, const char* text) {
    FILE *file = fopen(file_path, "a");
    if (file == NULL) {
        printf("Error opening the file for writing\n");
        return;
    }

    // Obtener la fecha y hora actual
    struct tm timeinfo = get_current_time();

    fprintf(file, "[%02d],%s,\n", timeinfo.tm_hour, text);

    fclose(file);

}

float* read_file(const char* file_path, int startLine, int *listSize) {
    FILE* file = fopen(file_path, "r");

    if (file == NULL) {
        perror("Error opening the file");
        return NULL;
    }

    float* values = (float*)malloc(*listSize * sizeof(float));
    if (values == NULL) {
        perror("Error allocating memory for the list");
        fclose(file);
        return NULL;
    }

    int i = 0;
    int linesCounter = 0; 
    char line[256]; // Número de caracteres que desea leer de cada linea, Ajusta el tamaño según tus necesidades
    //printf("Valor de startLines %d\n", startLine);
    while (fgets(line, sizeof(line), file) != NULL && i < *listSize) {
        if (linesCounter >= startLine) {

            if (strcmp(line, "nan\n") == 0) {
                values[i] = NAN;  // Representación de NaN en C
            } else {
                sscanf(line, "%f", &values[i]);
            }

            i++;
        }
        linesCounter++;
    }

    *listSize = linesCounter - startLine;
    
    fclose(file);
    //*listSize = i;

    return values;
}

struct DataEntry {
    struct tm timeinfo;
    float value;
};


void value_to_list(float *list, String value, int pos ){
    if (value.equalsIgnoreCase("nan")) {
        list[pos] = NAN;  // Representación de NaN en C
    } else {
        list[pos] = value.toFloat();
    }
}

// Función para convertir una cadena de fecha y hora a una estructura tm
int parse_date_time(const char *str, struct tm *timeinfo) {
    return sscanf(str, "[%d-%d-%d %d:%d:%d]", 
                  &timeinfo->tm_year, &timeinfo->tm_mon, &timeinfo->tm_mday,
                  &timeinfo->tm_hour, &timeinfo->tm_min, &timeinfo->tm_sec);
}

// Función para leer el file y obtener los datos
void read_data_from_file(const char *file_path) {
    FILE *file = fopen(file_path, "r");
    if (file == NULL) {
        printf("Error opening file for reading\n");
        return;
    }

    char line[256];
    struct DataEntry entry;

    while (fgets(line, sizeof(line), file) != NULL) {
        // Parsear la línea para obtener fecha, hora y valor float
        if (parse_date_time(line, &entry.timeinfo) == 6) {
            char *value_str = strchr(line, ']');
            if (value_str != NULL) {
                value_str += 2;  // Apuntar al espacio después del ']'
                sscanf(value_str, "%f", &entry.value);

                // Hacer algo con la entrada, por ejemplo, imprimir en la consola
                printf("Fecha: %04d-%02d-%02d %02d:%02d:%02d, Valor: %.5f\n",
                       entry.timeinfo.tm_year, entry.timeinfo.tm_mon, entry.timeinfo.tm_mday,
                       entry.timeinfo.tm_hour, entry.timeinfo.tm_min, entry.timeinfo.tm_sec,
                       entry.value);
            } else {
                printf("Error extracting float value from line: %s\n", line);
            }
        } else {
            printf("Error parsing the line: %s\n", line);
        }
    }

    fclose(file);
}

