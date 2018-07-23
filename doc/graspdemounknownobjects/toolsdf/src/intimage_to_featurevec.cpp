/*
 * David Fischinger -TUW
 * 15.11.2011
 *
 * USAGE:
 * 	- generate ListOfIIFilenames.txt in correct folder
 * 	- change all Parameters
 * 	- execute
 *
 *
 * input:
 *   Filenames (generated with: "ls pcd* -1 > ListOfIIFilenames.txt"
 *   executed in IntegralImages folder)
 *   with integral images and filename with features
 *
 * output:
 *   One file for each integral images including all feature values (1 value for each feature) is saved in the given output folder.
 *
 *   PARAMETERS:
 *
 *    HEIGHT 15 (14+1)
 *    WIDTH 15 (14+1)
 *    path =    "/home/grasp/David/GPDatabase/goodgps/twocams/intimages/";
 *    pathfull_list_of_filenames = path + "ListOfIIFilenames.txt";
 *    pathout = "/home/grasp/David/GPDatabase/goodgps/twocams/featurevec/";
 *    goodgps = true		indicates if features for good or bad GPs are calculated => label +1/-1 in output .txt
 *	  path =    "/opt/ros/privat/stacks/toolsdf/CeditFeatures_input.txt";       //file with Haar-Features
 */


//#include <boost/thread/thread.hpp>
#include <iostream>
#include <string.h>
#include <sstream>
#include <fstream>

//#include "CIntegralImage.cpp"
#include <time.h>
#include <CHaarFeature.h>

#define HEIGHT 15
#define WIDTH 15

using namespace std;
using namespace cv;



class CIntImage_to_Featurevec
{
public:
	CHaarFeature * currentfeature;	//number of regions
	vector<CHaarFeature> allfeatures;
	float intimagemat[HEIGHT][WIDTH];
	//PARAMETERS
	string path;
	string pathfull_list_of_filenames;
	string pathout;
	bool goodgps;


	void print_heights(float intimagemat[][WIDTH]);
	void read_features();
	void print_features();
	void write_featurevector(string outputpath);
	float calc_featurevalue(int nr_feat);


	//double calcFval(cv::Mat heightsIntegral, int pos_x, int pos_y);
	CIntImage_to_Featurevec(){
		currentfeature = new CHaarFeature(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.1,1.0,1.0,2.0);
		path =    "/home/grasp/David/GPDatabase/goodgps/twocams/intimages/";
		pathfull_list_of_filenames = path + "ListOfIIFilenames.txt";
		pathout = "/home/grasp/David/GPDatabase/goodgps/twocams/featurevec/";
		goodgps = true;
	}
};


void CIntImage_to_Featurevec::read_features()
{
	//PARAMETERS
	string path =    "/opt/ros/privat/stacks/toolsdf/CeditFeatures_input.txt";

 	ifstream file_features;
 	file_features.open(path.c_str());	//file_features: file with all features
	if (!file_features){
		cout << "\n PROBLEM opening feature file: " << path.c_str() << endl;
	}

	string line;
	int id_x = -1;
	getline(file_features, line);
	while (file_features.good())
	{
		int start = 0, end = 0;
		int reg_c[16];				//region_corners
		float reg_w[4];	//region_weights
		id_x++;
		for (int i = 0; i < 16; i++){
			end = line.find("\t", start);
			reg_c[i] = atoi(line.substr(start,end-start).c_str()) ;
			start = end+1;
		}
		for (int j = 0; j < 4; j++){
			end = line.find("\t", start);
			reg_w[j] = atof(line.substr(start,end-start).c_str()) ;
			start = end+1;
		}

		CHaarFeature * tmp_feature = new CHaarFeature(reg_c[0],reg_c[1],reg_c[2],reg_c[3], reg_c[4],reg_c[5],reg_c[6],reg_c[7],reg_c[8],reg_c[9],reg_c[10],reg_c[11],reg_c[12],reg_c[13],reg_c[14],reg_c[15],reg_w[0],reg_w[1],reg_w[2],reg_w[3]);
		this->allfeatures.push_back(*tmp_feature);

		getline(file_features, line);
	}
	file_features.close();
}


void CIntImage_to_Featurevec::print_features()
{

	for (int i = 0; i < this->allfeatures.size(); i++)
	{
		CHaarFeature curfeature = this->allfeatures.at(i);
		for (int j = 0; j < 16; j++){
			int reg_num = curfeature.regions.at(j);
			cout << reg_num << "\t";
		}
		for (int j = 0; j < 4; j++){
					float reg_w = curfeature.weights.at(j);
					cout << reg_w << "\t";
				}
		cout << i << "\n" ;
	}
}


void CIntImage_to_Featurevec::print_heights(float intimagemat[][WIDTH])
{
    //print heights matrix
	cout << "print integral image matrix:" << endl;
	for (int i = 0; i < HEIGHT; i++){  //rows
		for (int j = 0; j < WIDTH; j++){  //cols
			cout << intimagemat[i][j] << "\t";
		}
		cout << "\n";
	}
	cout << "\n david: ";
	cout << this->currentfeature->nr_reg << endl;
}

void CIntImage_to_Featurevec::write_featurevector(string outputpath)
{
    //open output file
	ofstream output_fv_file(outputpath.c_str());

	if (this->goodgps)
		output_fv_file << "+1";
	else
		output_fv_file << "-1";
	//for all features
	for (int nr_feat = 0; nr_feat < this->allfeatures.size(); nr_feat++){
		float featureval = calc_featurevalue(nr_feat);
		output_fv_file << " " << nr_feat+1 << ":" << featureval;
	}
	output_fv_file << "\n";
	output_fv_file.close();
}


float CIntImage_to_Featurevec::calc_featurevalue(int nr_feat)
{
	float returnval = 0;
	this->currentfeature = &this->allfeatures.at(nr_feat);
	for (int nr_reg = 0; nr_reg <4; nr_reg++){
		//corners of region
		int x1 = currentfeature->regions[nr_reg*4];
		int x2 = currentfeature->regions[nr_reg*4+1];
		int y1 = currentfeature->regions[nr_reg*4+2];
		int y2 = currentfeature->regions[nr_reg*4+3];
		float wgt = currentfeature->weights[nr_reg];	//weight of region

		if (( wgt == 0.0 ) or		//region not used (weight equal 0)
			( x2 < x1 ) or			//r.x2 < r.x1
			( y2 < y1 ) or			//r.y2 < r.y1
			(x2 == 0 and y2 == 0)) 	//region corners are (0,0,0,0)
			continue;

		returnval += wgt*(this->intimagemat[x2+1][y2+1] - this->intimagemat[x1][y2+1] -
						  this->intimagemat[x2+1][y1] + this->intimagemat[x1][y1]);
	}
	//cout << "feature, returnval: " << nr_feat << "  " << returnval << endl;
	return returnval;
}


int main (int argc, char** argv)
{
	CIntImage_to_Featurevec * ii_to_fv = new CIntImage_to_Featurevec();

	ii_to_fv->read_features();
	//ii_to_fv->print_features();

 	ifstream filenames_ii;
 	filenames_ii.open(ii_to_fv->pathfull_list_of_filenames.c_str());	//filename_ii (summary of integral images filenames)

 	int cnt =0;
	// loop which reads all intimages files
	string filename_cur_ii; //filename of current intimages .txt-file
 	getline(filenames_ii, filename_cur_ii);
	while (!filenames_ii.eof())
	{
		//float intimagemat[HEIGHT][WIDTH];
		cout << ++cnt << "\n" << filename_cur_ii << endl;
		stringstream ss_file_in;	//integral image file in
		stringstream ss_file_out;	//txt with feature vector for integral image
		ss_file_in << ii_to_fv->path << filename_cur_ii;
		ss_file_out << ii_to_fv->pathout << filename_cur_ii.substr(0,filename_cur_ii.length()-6) << "fv" << ".txt"; //insert "_fv" in name of output feature vector file

		//load grid
		ifstream file_cur_ii((ss_file_in.str()).c_str());
		if (!file_cur_ii){
			cout << "\n PROBLEM opening file: " << ss_file_in.str() << endl;
		}
		cout << "\n " << ss_file_in.str() << endl;
		string line;
		int id_x = -1;
		getline(file_cur_ii, line);
		while (file_cur_ii.good())
		{
	  		int start = 0, end = 0;
	  		id_x++;
	  		for (int id_y = 0; id_y < WIDTH; id_y++){
	 			end = line.find("\t", start);
	 			ii_to_fv->intimagemat[id_x][id_y] = atof(line.substr(start,end-start).c_str()) ;
	 			start = end+1;
	 		}
	  		getline(file_cur_ii, line);
		}

		cout << "\n\n";
		//ii_to_fv->print_heights(ii_to_fv->intimagemat);

		//print file with feature vector
		ii_to_fv->write_featurevector(ss_file_out.str());

		getline(filenames_ii, filename_cur_ii);
	}
	filenames_ii.close();
	return 0;
}

