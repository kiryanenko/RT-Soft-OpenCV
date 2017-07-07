// Task1_hist. Написать программу для расчета гистограмм для набора изображений, которые находятся в папке
// - Написать функктор, позволяющий упорядочить набор изображений по степени похожести (сравниваем на основе
// гистограмм) на образец при помощи функции std::sort
// https://varrunr.wordpress.com/2012/07/30/passing-extra-parameters-to-stl-sort/

#include <iostream>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

#define File_Name_Len 80

class Image {
    MatND m_hist;
    char m_name[File_Name_Len];

public:
    Image(const char *filename);
    Image(const Image &obj);
    const char *getName() const;
    MatND getHist() const;
};

Image::Image(const char *filename) {
    strcpy(m_name, filename);
    int histSize = 256; // bin size
    float range[] = { 0, 255 };
    const float *ranges[] = { range };
    Mat img = imread(filename, 0);
    calcHist( &img, 1, 0, Mat(), m_hist, 1, &histSize, ranges, true, false ); // Calculate histogram
}

const char *Image::getName() const {
    return m_name;
}

MatND Image::getHist() const {
    return m_hist;
}

Image::Image(const Image &obj) {
    strcpy(m_name, obj.getName());
    m_hist = obj.getHist();
}

// Функтор сортировки
class Similar {
    MatND m_cmpHist;

public:
    Similar(const char *cmpImage);
    bool operator() (Image &a, Image &b);
};

Similar::Similar(const char *cmpImage) {
    Mat grey = imread(cmpImage, 0);
    int histSize = 256; // bin size
    float range[] = { 0, 255 };
    const float *ranges[] = { range };
    calcHist( &grey, 1, 0, Mat(), m_cmpHist, 1, &histSize, ranges, true, false ); // Calculate histogram
}

bool Similar::operator()(Image &a, Image &b) {
    double cmpA = compareHist(a.getHist(), m_cmpHist, CV_COMP_CORREL);
    double cmpB = compareHist(b.getHist(), m_cmpHist, CV_COMP_CORREL);
    return cmpA > cmpB;
}

int main() {
    char imagesFileNames[][File_Name_Len] = {
            "images/box.JPG",
            "images/chair.JPG",
            "images/computer1.JPG",
            "images/computer2.JPG",
            "images/fire_extinguisher.JPG",
            "images/hanger.JPG",
            "images/keyboard.JPG",
            "images/monitor.JPG",
            "images/switches.JPG"
    };
    size_t count = sizeof imagesFileNames / File_Name_Len;

    vector<Image> images;
    for (size_t i = 0; i < count; ++i) {
        images.push_back( Image(imagesFileNames[i]) );
    }

    Similar similaritySort("cmpImage.JPG");
    sort(images.begin(), images.end(), similaritySort);

    for (size_t i = 0; i < count; ++i) {
        cout << images[i].getName() << endl;
    }

    return 0;
}