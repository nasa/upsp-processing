/*
 * kulites.cpp
 *
 * Created on: October 18, 2017
 *     Author: jmpowel2
 */

#include "kulites.h"

/*****************************************************************************/
upsp::VKulDirection upsp::parse_virtual_kulite_direction(std::string dir) {

    VKulDirection dir_out;
    if ((dir == "top") || (dir == "t")) {
        dir_out = TOP;
    } else if ((dir == "bottom") || (dir == "b")) {
        dir_out = BOTTOM;
    } else if ((dir == "left") || (dir == "l")) {
        dir_out = LEFT;
    } else if ((dir == "right") || (dir == "r")) {
        dir_out = RIGHT;
    } else if (dir == "upstream") {
        dir_out = UPSTREAM;
    } else if (dir == "downstream") {
        dir_out = DOWNSTREAM;
    } else {
        throw(std::invalid_argument("Unknown virtual kulite direction"));
    }
    return dir_out;
}

/*****************************************************************************/
void upsp::locate_virtual_kulites(std::vector<Kulite>& kuls, 
        VKulDirection dir/*=LEFT*/, double stand_off/*=1.5*/,
        int set_size/*=-1*/) {
    
    cv::Point2d pt_dir(0.0,0.0);
    if (dir == RIGHT) {
        pt_dir.x = 1;
    } else if (dir == LEFT) {
        pt_dir.x = -1;
    } else if (dir == TOP) {
        pt_dir.y = -1;
    } else if (dir == BOTTOM) {
        pt_dir.y = 1;
    } else if ( (dir == UPSTREAM) || (dir == DOWNSTREAM) ) {
        throw(std::invalid_argument("Cannot parse upstream/downstream without" + 
                std::string(" additional inputs, use other") + 
                " locate_virtual_kulites function"));
    } else {
        throw(std::invalid_argument("Unknown virtual kulite direction"));
    }

    upsp::locate_virtual_kulites(kuls, pt_dir, stand_off, set_size);

}

/*****************************************************************************/
void upsp::locate_virtual_kulites(std::vector<Kulite>& kuls, cv::Point2d dir, 
        double stand_off/*=1.5*/, int set_size/*=-1*/) {

    // normalize direction
    if (cv::norm(dir) > 0.0) {
        dir = dir / cv::norm(dir);
    }

    int x_pos,y_pos;
    for (int i=0; i < kuls.size(); i++) {

        // Fix kulite size
        if (set_size > 0) {
            kuls[i].size = set_size;
        } else {
            kuls[i].size = std::round(kuls[i].diameter);
        }   

        // Set offset of virtual kulite
        cv::Point2d offset(-0.5,-0.5);
        offset += stand_off*dir;

        // Set Location of virtual kulite       
        x_pos = std::round(kuls[i].uv.x + kuls[i].size*offset.x);
        y_pos = std::round(kuls[i].uv.y + kuls[i].size*offset.y);

        kuls[i].top_left = cv::Point2d(x_pos,y_pos);
    }

}

/*****************************************************************************/
void upsp::extract_virtual_kulites(const cv::Mat& src, std::vector<Kulite>& kuls) {

    cv::Mat roi;
    for (int i=0; i < kuls.size(); ++i) {
        roi = src.colRange(kuls[i].top_left.x, kuls[i].top_left.x + 
                kuls[i].size+1).rowRange(kuls[i].top_left.y, 
                kuls[i].top_left.y+kuls[i].size+1);
        kuls[i].value = mean(roi)[0];
    }
}

/*****************************************************************************/
bool upsp::in_frame(Kulite& kul, cv::Size sz) {
    return ( (kul.top_left.x >= 0) && (kul.top_left.x+kul.size < sz.width) &&
            (kul.top_left.y >= 0) && (kul.top_left.y+kul.size < sz.height) );
}

