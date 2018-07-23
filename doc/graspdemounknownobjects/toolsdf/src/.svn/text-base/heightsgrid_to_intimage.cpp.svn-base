/*
 * David Fischinger -TUW
 * 11.11.2011
 *
 * input:
 *   Filenames (generated with: ls pcd* -1 > ListOfHeightsGridFilenames.txt
 *   executed in heights folder)
 *   with height grids of pointclouds are read
 *
 * output:
 *   Files with integral images of these grids are saved in the given folder.
 *
 *   PARAMETERS:
 *
 *    TGHEIGHT 600
 *    TGWIDTH 800
 *    HEIGHT 14
 *    WIDTH 14
 *    path =    "/home/grasp/David/GPDatabase/badgps/twocams/heights/";
 *    pathfull_list_of_filenames = path + "ListOfHeightsGridFilenames.txt";
 *    pathout = "/home/grasp/David/GPDatabase/badgps/twocams/intimages/";
 *
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string.h>
#include <sstream>
#include <fstream>

#include <v4r/TomGine/tgTomGine.h>
//#include "CIntegralImage.cpp"
//#include "CHaarFeature.h"
#include <time.h>
#include <iomanip>

#define TGHEIGHT 600
#define TGWIDTH 800
#define HEIGHT 14
#define WIDTH 14

using namespace std;
using namespace TomGine;
using namespace cv;



void print_heights(float heightsgrid[][WIDTH])
{
    //print heights matrix
	cout << "print heights matrix:" << endl;
	for (int i = 0; i < HEIGHT; i++){  //rows
		for (int j = 0; j < WIDTH; j++){  //cols
			cout << heightsgrid[i][j] << "\t";
		}
		cout << "\n";
	}
}

// print integral images of heights grid
/*void CPC_To_HeightGrid::print_intheights(float heightsmat[][WIDTH])
{
	cout << "print integral height matrix on screen\n";

    //print heights matrix
	for (int row = 0; row < 1+2*this->nr_rows; row++){  //rows
		for (int col = 0; col < 1+2*this->nr_rows; col++){  //cols
			cout << setw(13) << ((double*)(this->heightsIntegral.ptr() + this->heightsIntegral.step*row))[col];
		}
		cout << "\n";
	}
}*/


//calculates integral image of heights grid
void calc_and_save_intheights(float heightsmat[][WIDTH], string fullPathIIOut)
{


    Mat heightsIntegral;
    heightsIntegral = Mat(HEIGHT+1, WIDTH+1, CV_32FC1);
	Mat const cvheightsmat = cv::Mat(HEIGHT, WIDTH, CV_32FC1, heightsmat);
	cv::integral(cvheightsmat, heightsIntegral,CV_32FC1);

	cout << "\n print integral height matrix on screen\n";


    //print ii matrix
	for (int row = 0; row < 1+WIDTH; row++){  //rows
		for (int col = 0; col < 1+HEIGHT; col++){  //cols
			cout << setw(13) << ((float*)(heightsIntegral.ptr() + heightsIntegral.step*row))[col];
		}
		cout << "\n";
	}

	//save integral images matrix
	ofstream out_file( fullPathIIOut.c_str() );
    //save integral image as table/matrix
	for (int row = 0; row < HEIGHT+1; row++){  //rows
		for (int col = 0; col < WIDTH+1; col++){  //cols
			out_file << ((float*)(heightsIntegral.ptr() + heightsIntegral.step*row))[col] << "\t";
		}
		out_file << "\n";
	}
	out_file.close();
	cout << "This file was saved: " << fullPathIIOut << endl;
}



int main (int argc, char** argv)
{
	//PARAMETERS
	int nr_rows = HEIGHT;
	int nr_cols = WIDTH;
	string path =    "/home/grasp/David/GPDatabase/badgps/twocams/heights/";
	string pathfull_list_of_filenames = path + "ListOfHeightsGridFilenames.txt";
	string pathout = "/home/grasp/David/GPDatabase/badgps/twocams/intimages/";


 	ifstream filenames_hg;
 	filenames_hg.open(pathfull_list_of_filenames.c_str());	//filname_hg (summary of heights grid filenames)

 	int cnt =0;
	// loop which reads all heights grid files
	string filename_cur_hg; //filename of current heights grid .txt-file
 	getline(filenames_hg, filename_cur_hg);
	while (!filenames_hg.eof())
	{
		float heightsgrid[HEIGHT][WIDTH];
		cout << ++cnt << "\n" << filename_cur_hg << endl;
		stringstream ss_file_in;	//heights gird file in
		stringstream ss_file_out;	//txt with maxheights grid
		ss_file_in << path << filename_cur_hg;
		ss_file_out << pathout << filename_cur_hg.substr(0,filename_cur_hg.length()-4) << "_ii" << ".txt"; //insert "_ii" in name of output integral image file

		//load grid
		ifstream file_cur_hg((ss_file_in.str()).c_str());
		if (!file_cur_hg){
			cout << "\n PROBLEM opening file: " << ss_file_in.str() << endl;
		}
		cout << "\n " << ss_file_in.str() << endl;
		string line;
		int id_x = -1;
		getline(file_cur_hg, line);
		while (file_cur_hg.good())
		{
	  		int start = 0, end = 0;
	  		id_x++;
	  		for (int id_y = 0; id_y < WIDTH; id_y++){
	 			end = line.find("\t", start);
	 			heightsgrid[id_x][id_y] = atof(line.substr(start,end-start).c_str()) ;
	 			start = end+1;
	 		}
	  		getline(file_cur_hg, line);
		}

		//save_heightsgrid(heightsgrid, ss_file_out.str());
		cout << "\n\n";
		calc_and_save_intheights(heightsgrid, ss_file_out.str());
		//print_heights(heightsgrid);
		//show_heights(heightsgrid);
		getline(filenames_hg, filename_cur_hg);
	}
	filenames_hg.close();
	return 0;
}

