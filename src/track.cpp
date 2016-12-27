#include "track.h"
#include <fstream>
#include <sstream>
#include<iomanip>
#include <iostream>

namespace tracker{

    void Track::LoadImages(const string &path_to_dir, vector <string> &images_left,
                           vector <string> &images_right) {
        ifstream fTimes;
        unsigned int frames=0;
        // get size of frames from times.txt
        string strPathTimeFile = path_to_dir + "/times.txt";
        fTimes.open(strPathTimeFile.c_str());
        while (!fTimes.eof()) {
            string s;
            getline(fTimes, s);
            frames++;
        }
        // make file lists
        string strPrefixLeft = path_to_dir + "/left/";
        string strPrefixRight = path_to_dir + "/right/";

        images_left.resize(frames);
        images_right.resize(frames);

        for (int i = 0; i < frames; i++) {
            stringstream ss;
            ss << setfill('0') << setw(6) << i;
            images_left[i] = strPrefixLeft + ss.str() + ".jpg";
            images_right[i] = strPrefixRight + ss.str() + ".jpg";
        }
    }


}
