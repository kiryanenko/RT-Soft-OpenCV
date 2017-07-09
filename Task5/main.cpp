// Task5_hist. Написать программу для расчета характеристичесих точек для набора изображений, которые находятся в папке
// - Написать функктор, позволяющий упорядочить набор изображений по степени похожести
// (сравниваем на основе характеристичесих точек) на образец при помощи функции std::sort


#include <iostream>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
using namespace std;
using namespace cv;

cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create();


Mat createDescriptors(const char * imgFilename) {
    Mat descriptors, img = imread(imgFilename);
    vector<KeyPoint> keypoints;
    f2d->detectAndCompute( img, noArray(), keypoints, descriptors );
    return descriptors;
}


class Image {
    Mat m_descriptors;
    string m_name;

public:
    Image(const char *filename);
    string getName() const;
    Mat getDescriptors() const;
};

Image::Image(const char *filename) {
    m_name = filename;
    m_descriptors = createDescriptors(filename);
}

string Image::getName() const {
    return m_name;
}

Mat Image::getDescriptors() const {
    return m_descriptors;
}


// Функтор сортировки
class Similar {
    Mat m_cmpDescriptors;

private:
    std::vector< DMatch > getGoodMatches(Mat descr);

public:
    Similar(const char *cmpImage);
    bool operator() (Image &a, Image &b);
};

Similar::Similar(const char *cmpImage) {
    m_cmpDescriptors = createDescriptors(cmpImage);
}

bool Similar::operator()(Image &a, Image &b) {
    // Сравниваю по количеству хороших точек
    return getGoodMatches(a.getDescriptors()) > getGoodMatches(b.getDescriptors());
}

std::vector< DMatch > Similar::getGoodMatches(Mat descr) {
    FlannBasedMatcher matcher; // FLANN - Fast Library for Approximate Nearest Neighbors
    vector< vector< DMatch> > matches;
    matcher.knnMatch( m_cmpDescriptors, descr, matches, 2 ); // find the best 2 matches of each descriptor
    vector< DMatch > goodMatches;
    for (int k = 0; k < std::min(m_cmpDescriptors.rows - 1, (int)matches.size()); k++) {
        if ((matches[k][0].distance < 0.6 * (matches[k][1].distance)) && ((int) matches[k].size() <= 2 &&
                                                                          (int) matches[k].size() > 0)) {
            // take the first result only if its distance is smaller than 0.6*second_best_dist
            // that means this descriptor is ignored if the second distance is bigger or of similar
            goodMatches.push_back(matches[k][0]);
        }
    }
    return goodMatches;
}


int main() {
    std::vector<Image> images = {
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

    Similar similaritySort("cmpImage.JPG");
    sort(images.begin(), images.end(), similaritySort);

    for (const auto& image : images) {
        cout << image.getName() << endl;
    }

    return 0;
}