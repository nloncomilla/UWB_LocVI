#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <numeric>
#include <algorithm>
#include "globales.h"


double errores[4][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

double r1_error = 0.0;
double r2_error = 0.0;
double r3_error = 0.0;

std::vector<float> parseSerialData(const std::string& data);

void realizarPruebaDistancia(const std::string& mensaje, double distancia_metros) {
    std::cout << mensaje << " y presione 'y' para continuar: ";
    std::string respuesta;
    std::cin >> respuesta;

    std::transform(respuesta.begin(), respuesta.end(), respuesta.begin(), ::tolower);

    if (respuesta != "yes" && respuesta != "y" && respuesta != "si" && respuesta != "s") {
        std::cout << "Calibracion para " << distancia_metros << "m cancelada.\n";
        return;
    }

    boost::asio::io_service io;
    boost::asio::serial_port serial(io, "COM3");
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

    std::string message_buffer;
    std::vector<float> r0_values, r1_values, r2_values;
    const int num_lecturas = 100;
    int lecturas_tomadas = 0;

    std::cout << "Iniciando la toma de lecturas para " << distancia_metros << " metros...\n";

    while (lecturas_tomadas < num_lecturas) {
        char c;
        std::size_t bytes_read = serial.read_some(boost::asio::buffer(&c, 1));
        if (bytes_read > 0) {
            message_buffer += c;
            if (c == '\n') {
                if (message_buffer.find("AT+RANGE=") != std::string::npos) {
                    std::vector<float> distancias = parseSerialData(message_buffer);

                    if (!distancias.empty() && distancias.size() >= 3) {
                        r0_values.push_back(distancias[0]);
                        r1_values.push_back(distancias[1]);
                        r2_values.push_back(distancias[2]);

                        lecturas_tomadas++;
                        std::cout << "Lectura " << lecturas_tomadas << ": r0=" << distancias[0] << ", r1=" << distancias[1] << ", r2=" << distancias[2] << std::endl;

                        if (lecturas_tomadas >= num_lecturas) {
                            break;
                        }
                    }
                }
                message_buffer.clear();
            }
        }
    }

    auto calcularMedia = [](const std::vector<float>& valores) {
        return std::accumulate(valores.begin(), valores.end(), 0.0f) / valores.size();
    };

    float media_r0 = calcularMedia(r0_values);
    float media_r1 = calcularMedia(r1_values);
    float media_r2 = calcularMedia(r2_values);

    double r0_error = (media_r0 - distancia_metros) * 100;
    double r1_error = (media_r1 - distancia_metros) * 100;
    double r2_error = (media_r2 - distancia_metros) * 100;

    int index = -1;
    if (distancia_metros == 1.0) {
        index = 0;
    } else if (distancia_metros == 2.0) {
        index = 1;
    } else if (distancia_metros == 3.0) {
        index = 2;
    } else if (distancia_metros == 5.0) {
        index = 3;
    }

    if (index != -1) {
        errores[index][0] = r0_error / 100;
        errores[index][1] = r1_error / 100;
        errores[index][2] = r2_error / 100;
    }

    r0_error = r0_error / 100;
    r1_error = r1_error / 100;
    r2_error = r2_error / 100;

    std::ofstream f_out("./data/resultados_calibracion.txt", std::ios::app);
    if (f_out.is_open()) {
        f_out << "Resultados de la calibracion para " << distancia_metros << " metros (media de 100 lecturas):\n";
        f_out << "Media r0: " << media_r0 << " cm, Error r0: " << r0_error << " cm\n";
        f_out << "Media r1: " << media_r1 << " cm, Error r1: " << r1_error << " cm\n";
        f_out << "Media r2: " << media_r2 << " cm, Error r2: " << r2_error << " cm\n";
        f_out.close();
    }

    std::cout << "Calibracion para " << distancia_metros << " metros finalizada. Resultados:\n";
    std::cout << "Media r0: " << media_r0 << " cm, Error r0: " << r0_error << " cm\n";
    std::cout << "Media r1: " << media_r1 << " cm, Error r1: " << r1_error << " cm\n";
    std::cout << "Media r2: " << media_r2 << " cm, Error r2: " << r2_error << " cm\n";
}

void realizarCalibracion() {
    std::cout << "Bienvenido a la etapa de calibracion.\n";
    std::cout << "Se tomaran lecturas a distancias de 1m, 2m, 3m y 5m.\n";
    std::cout << "¿Desea iniciar la calibracion? (yes/y/si/s): ";

    std::string respuesta;
    std::cin >> respuesta;

    std::transform(respuesta.begin(), respuesta.end(), respuesta.begin(), ::tolower);

    if (respuesta != "yes" && respuesta != "y" && respuesta != "si" && respuesta != "s") {
        std::cout << "Calibracion cancelada.\n";
        return;
    }

    realizarPruebaDistancia("Por favor, coloque los anchors a 1 metro de distancia", 1.0);
    realizarPruebaDistancia("Por favor, coloque los anchors a 2 metros de distancia", 2.0);
    realizarPruebaDistancia("Por favor, coloque los anchors a 3 metros de distancia", 3.0);
    realizarPruebaDistancia("Por favor, coloque los anchors a 5 metros de distancia", 5.0);

    std::cout << "Calibracion completada. Los errores se han guardado en 'resultados_calibracion.txt'.\n";
}
