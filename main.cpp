#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <regex>
#include "globales.h"
#include "calibraciones.cpp"

// Posicion antenas
Eigen::Vector2d anchor0(0.0, 0.0);  // A0 en (0, 0)
Eigen::Vector2d anchor1(0.0, 3.0);  // A1 en (0, 3)m
Eigen::Vector2d anchor2(3.5, 3.0);  // A2 en (3.5, 3)



// Función para convertir posiciones en metros a píxeles, y trasladarlas para centrarlas
cv::Point meterToPixel(const Eigen::Vector2d& position, int window_width, int window_height) {
    const double scale = 100.0;  // Escala 1:100 metro:píxeles
    int offsetX = window_width / 4;  // Se centra el eje de coordenadas en el centro del segundo cuadrante
    int offsetY = window_height / 4; // del plano. (Ubicación de A0)

    int x = static_cast<int>(position.x() * scale) + offsetX;
    int y = static_cast<int>(position.y() * scale) + offsetY;

    return cv::Point(x, y);
}


void drawAnchors(cv::Mat& frame, const std::vector<Eigen::Vector2d>& anchors, int window_width, int window_height) {
    const int radius = 10;
    const cv::Scalar anchorColor(0, 0, 255);  // Rojo para las Antenas
    const cv::Scalar textColor(255, 255, 255);  // Blanco para el texto

    
    std::vector<std::string> anchorNames = {"A0", "A1", "A2"};

    for (int i = 0; i < anchors.size(); ++i) {
        cv::Point anchorPoint = meterToPixel(anchors[i], window_width, window_height);

        
        cv::circle(frame, anchorPoint, radius, anchorColor, -1);

        
        std::string label = anchorNames[i] + " (" + std::to_string(anchors[i].x()) + ", " + std::to_string(anchors[i].y()) + ")";
        cv::putText(frame, label, cv::Point(anchorPoint.x + 15, anchorPoint.y - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1);
    }
}


void drawTag(cv::Mat& frame, const Eigen::Vector2d& tagPosition, int window_width, int window_height) {
    const int radius = 8;
    const cv::Scalar tagColor(0, 255, 0);  // Verde para el Tag sin LSQ

    cv::Point tagPoint = meterToPixel(tagPosition, window_width, window_height);

    cv::circle(frame, tagPoint, radius, tagColor, -1);

    std::string coords = "(" + std::to_string(tagPosition.x()) + ", " + std::to_string(tagPosition.y()) + ")";
    cv::putText(frame, coords, cv::Point(tagPoint.x + 15, tagPoint.y - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, tagColor, 1);
}


void drawTagLQS(cv::Mat& frame, const Eigen::Vector2d& tagPosition, int window_width, int window_height) {
    const int radius = 8;
    const cv::Scalar tagColor(245, 224, 39);  // Celeste para el Tag con LQS

    cv::Point tagPoint = meterToPixel(tagPosition, window_width, window_height);

    cv::circle(frame, tagPoint, radius, tagColor, -1);

    std::string coords = "(" + std::to_string(tagPosition.x()) + ", " + std::to_string(tagPosition.y()) + ")";
    cv::putText(frame, coords, cv::Point(tagPoint.x + 15, tagPoint.y - 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, tagColor, 1);
}


void drawGrid(cv::Mat& frame) {
    const int scale = 100;
    const cv::Scalar gridColor(100, 100, 100);

    for (int i = 0; i <= frame.cols; i += scale) {
        cv::line(frame, cv::Point(i, 0), cv::Point(i, frame.rows), gridColor, 1);
    }
    for (int i = 0; i <= frame.rows; i += scale) {
        cv::line(frame, cv::Point(0, i), cv::Point(frame.cols, i), gridColor, 1);
    }
}


// Función equivalente a 'three_point'
Eigen::Vector2d three_point(double x1, double y1, double x2, double y2, double r1, double r2) {
    double temp_x = 0.0;
    double temp_y = 0.0;


    double p2p = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
    p2p = std::sqrt(p2p);


    if (r1 + r2 <= p2p) {
        temp_x = x1 + (x2 - x1) * r1 / (r1 + r2);
        temp_y = y1 + (y2 - y1) * r1 / (r1 + r2);
    } else {
        double dr = p2p / 2 + (r1 * r1 - r2 * r2) / (2 * p2p);
        temp_x = x1 + (x2 - x1) * dr / p2p;
        temp_y = y1 + (y2 - y1) * dr / p2p;
    }

    return Eigen::Vector2d(temp_x, temp_y);
}

// Función para realizar la trilateración
Eigen::Vector2d trilaterate(const double& r0, const double& r1, const double& r2,
                            const Eigen::Vector2d& anchor0,
                            const Eigen::Vector2d& anchor1,
                            const Eigen::Vector2d& anchor2) {

    Eigen::Vector2d p_01 = three_point(anchor0.x(), anchor0.y(), anchor1.x(), anchor1.y(), r0, r1);
    Eigen::Vector2d p_02 = three_point(anchor0.x(), anchor0.y(), anchor2.x(), anchor2.y(), r0, r2);
    Eigen::Vector2d p_12 = three_point(anchor1.x(), anchor1.y(), anchor2.x(), anchor2.y(), r1, r2);

    // Promediar las tres posiciones calculadas
    Eigen::Vector2d average_position = (p_01 + p_02 + p_12) / 3.0;

    return average_position;
}


// Función de trilateración con LSQ
Eigen::Vector2d trilaterateLeastSquares(const double& r0, const double& r1, const double& r2, 
                                        const Eigen::Vector2d& anchor0, 
                                        const Eigen::Vector2d& anchor1, 
                                        const Eigen::Vector2d& anchor2,
                                        const Eigen::Vector3d& weights) {
    Eigen::Matrix2d A;
    Eigen::Vector2d b;

    A << 2 * (anchor1.x() - anchor0.x()) * weights(0), 2 * (anchor1.y() - anchor0.y()) * weights(0),
         2 * (anchor2.x() - anchor1.x()) * weights(1), 2 * (anchor2.y() - anchor1.y()) * weights(1);

    b << (r0 * r0 - r1 * r1 + anchor1.squaredNorm() - anchor0.squaredNorm()) * weights(0),
         (r1 * r1 - r2 * r2 + anchor2.squaredNorm() - anchor1.squaredNorm()) * weights(1);

    Eigen::Vector2d position;

    Eigen::Matrix2d A_transpose_A = A.transpose() * A;

    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    double regularization_term = 1e-8; 
    position = (A_transpose_A + regularization_term * I).inverse() * A.transpose() * b;

    return position;
}

// Función para parsear datos seriales y extraer las distancias
std::vector<float> parseSerialData(const std::string& data) {
    std::vector<float> distances(8, 0.0f); 
    try {
        std::regex rangeRegex(R"(\brange:\(([\d.,]+)\))"); 
        std::smatch match;

        if (std::regex_search(data, match, rangeRegex)) {
            std::string rangeData = match[1]; 

            std::stringstream rangeStream(rangeData);
            std::string item;
            int i = 0;

            // Parsear valores de range
            while (std::getline(rangeStream, item, ',') && i < 8) {
                distances[i] = std::stof(item) ;
                i++;
            }
        }
    } catch (...) {
        std::cerr << "Error al parsear el mensaje serial." << std::endl;
    }

    return distances;
}

int main() {
    
    // Tamaño ventana
    const int window_width = 800;
    const int window_height = 600;

    realizarCalibracion(); // Código previo

    boost::asio::io_service io;
    boost::asio::serial_port serial(io, "COM3");
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

    // Archivo de salida
    std::ofstream f_out("./data/posiciones_tag.txt");
    if (!f_out.is_open()) {
        std::cerr << "Error al abrir el archivo de salida." << std::endl;
        return 1;
    }

    double r0 = -1.0, r1 = -1.0, r2 = -1.0;

    std::string message_buffer;
    cv::Mat frame(window_height, window_width, CV_8UC3, cv::Scalar(255, 255, 255));


    while (true) {
        char c;

        while (serial.read_some(boost::asio::buffer(&c, 1)) > 0) {
            message_buffer += c;
            if (c == '\n') {
               
                if (message_buffer.find("range:") != std::string::npos) {
                    std::vector<float> distances = parseSerialData(message_buffer);

                    if (!distances.empty() && distances.size() >= 3) {  // Verificar que hay al menos 3 distancias
                        r0 = distances[0];
                        r1 = distances[1];
                        r2 = distances[2];

                        std::cout << "rangos: (" << r0 << "," << r1 << ", " << r2 << ")" << std::endl;
                        
                        // Comprobar que las antenas estan conectadas
                        if (r0 > 0 && r1 > 0 && r2 > 0) {

                            r0 -= errores[0][0];  
                            r1 -= errores[0][1];  
                            r2 -= errores[0][2];  

                            std::cout << "rangos: (" << r0 << "," << r1 << ", " << r2 << ")" << std::endl;

                            Eigen::Vector3d weights(1, 1, 1); 

                            
                            if (r0 > 300.0) weights(0) = 0.5; //reduce el peso cuando r > 300
                            if (r1 > 300.0) weights(1) = 0.5;  
                            if (r2 > 300.0) weights(2) = 0.5; 

                            r0 = r0 / 100.0;
                            r1 = r1 / 100.0;
                            r2 = r2 / 100.0;

                            Eigen::Vector2d tag_position = trilaterate(r0, r1, r2, anchor0, anchor1, anchor2);

                            f_out << "Posicion del tag: (" << tag_position.x() << ", " << tag_position.y() << ")" << std::endl;
                            std::cout << "Posicion del tag: (" << tag_position.x() << ", " << tag_position.y() << ")" << std::endl;

                            Eigen::Vector2d tag_positionLQS = trilaterateLeastSquares(r0, r1, r2, anchor0, anchor1, anchor2, weights);

                            f_out << "Posicion del tag (LQS): (" << tag_positionLQS.x() << ", " << tag_positionLQS.y() << ")" << std::endl;
                            std::cout << "Posicion del tag (LQS): (" << tag_positionLQS.x() << ", " << tag_positionLQS.y() << ")" << std::endl;

                            frame.setTo(cv::Scalar(0, 0, 0));  // Fondo negro

                            std::vector<Eigen::Vector2d> anchors = { anchor0, anchor1, anchor2 };

                            drawGrid(frame);

                            drawAnchors(frame, anchors, window_width, window_height);
                            drawTag(frame, tag_position, window_width, window_height);
                            drawTagLQS(frame, tag_positionLQS, window_width, window_height);


                            cv::imshow("UWB_LocUI", frame);
                            cv::waitKey(1);
                        }
                    }
                } else {

                    f_out << "Mensaje recibido (sin procesar): " << message_buffer << std::endl;
                }

                message_buffer.clear();
            }
        }
    }
    f_out.close();

    return 0;
}