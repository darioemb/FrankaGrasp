#include "common.h"

void loadRects(vector<Rect> &rects, const char* posFile, const char* negFile) {
	Rect r;
	rects.clear();
	printf("posFile = %s, negFile = %s\n", posFile, negFile);
	// load posRect from file
	ifstream fin(posFile);
	while (fin >> r) {
	    r.isPos = true;
	    rects.push_back(r);
	}
	fin.close();
	if (negFile != NULL) {
		fin.open(negFile, ifstream::in);
		while (fin >> r) {
			r.isPos = false;
			rects.push_back(r);
		}
		fin.close();
	}
}

int getdir (string dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        string s = string(dirp->d_name);
		/*============changed by Yun============*/
        //if (s.substr(0,4) == "left" && s.substr(8,9999) == ".pgm")
        if (s.substr(0,3) == "pcd" && s.substr(s.size()-4,4) == ".png")
            files.push_back(s);
    }
    
    sort(files.begin(), files.end());
    closedir(dp);
    return 0;
}

vector<Rect> ScoredRect_to_Rect(vector<ScoredRect> sr) {
  vector<Rect> r;
  r.reserve(sr.size());
  for (unsigned int i=0;i<sr.size();i++) {
    r.push_back(sr[i].rect);
  }
  return r;
}

