#include <iostream>
#include <stdlib.h>
#include <fstream>
#include "extractTrainingData.h"
#include "testModel.h"
#include "common.h"
#include <vector>
#include "calcRectIntersection.h"
#include "thr_pool.h"

using namespace std;

#define APPEND_RECTS true
#define N_RECTS_TO_ADD 50
#define ADD_TO_APPENDED_RECTS false

bool EVAL_TEST_SET = true;
bool GENERATE_NEW_TRAINING = false;
bool MODEL_PRE_TRAINED = false;
bool EARLY_STOP = false;
int N_CYCLES_START = 3;
int N_CYCLES = N_CYCLES_START + 1;

string trainingFile = "boostedTraining.txt";
string sortedTrainingFile = "sortedBoostedTraining.txt";
const char *dirPrefix = "/misc/projects/grasping_data/11-6/";
const char *labelsPrefix = "/misc/projects/grasping_data/11-6/new_labels/";
string trainVsTestFile = "train_test_split.txt";
string trainVsTestCVFile = "train_test_split_cv.txt";
string modelFilePrefix = "model_rank";
string statusFile = "boostStatus.txt";
string appendedRects = "appendedRects.txt";
string sortedAppendedRects = "sortedAppendedRects.txt";

void readBgMap(const char* filename, vector<int> &fNum, vector<string> &bgFilenames);

void readTrainTestSplit(const char *file, vector<int> fNum, vector<int> &trainingSet, vector<int> &testSet) {
	ifstream in; in.open(file);
	int n, isTest;
	trainingSet.clear();
	testSet.clear();
	while (in >> n >> isTest) {
		int k=0;
		while (fNum[k] != n) {
			k++;
		}
		(isTest ? testSet : trainingSet).push_back(k);
	}
}

void readTrainTestSplitCV(const char *file, vector<int> fNum, vector<int> &trainingSet, vector<int> &testSet) {
	ifstream in; in.open(file);
	int n, isTest;
	trainingSet.clear();
	testSet.clear();
	while (in >> n >> isTest) {
		int k=0;
		while (fNum[k] != n) {
			k++;
		}
		if (isTest == 1)
      testSet.push_back(k);
		else if (isTest == 0)
      trainingSet.push_back(k);
	}
}

typedef struct {
	int fileNum;
	vector<Rect> posRects;
  unsigned int nPosRects;
	vector<double> result;
	string dir;
  ofstream *of;
} arg_extract_rect_t;
arg_extract_rect_t *arg_extract_rect(
	int fileNum,
	vector<Rect> posRects,
  unsigned int nPosRects,
	vector<double> result,
	string dir,
  ofstream *of
  ) {
  arg_extract_rect_t  *t = new arg_extract_rect_t();
  t->fileNum = fileNum;
  
  t->posRects.reserve(posRects.size());
  for (int i=0;i<posRects.size();i++)
    t->posRects.push_back(posRects[i]);
  
  t->nPosRects = nPosRects;
  
  t->result.reserve(result.size());
  for (int i=0;i<result.size();i++)
    t->result.push_back(result[i]);
  
  if (t->posRects.size() != t->nPosRects + t->result.size()) {
    cout << "ERROR!! SIZES ARE WRONG!\n";
  }
  
  t->dir = dir;
  t->of = of;
  return t;
}
pthread_mutex_t mutex_extract_rect;
void *work_extract_rect(void *in) {
	arg_extract_rect_t *t = (arg_extract_rect_t*) in;
	(void)pthread_mutex_lock(&mutex_extract_rect);
  printf("%d %d %d %d %s\n",t->fileNum, t->posRects.size(), t->nPosRects, t->result.size(), t->dir.c_str());
	(void)pthread_mutex_unlock(&mutex_extract_rect);
  vector< vector<float> > all_fv = extractTrainingData(t->dir, t->fileNum, t->posRects);
  // cout << "done all_fv" << endl;
	(void)pthread_mutex_lock(&mutex_extract_rect);
  for (unsigned int k=0;k<all_fv.size();k++) {
    // of << (rects[k].isPos?"+1":"-1") << " qid:" << (fileNum+1);
    *(t->of) << (k < t->nPosRects ? 1: t->result[k-t->nPosRects]) << " qid:" << (t->fileNum+1);
    for (unsigned int i=0;i<all_fv[k].size();i++) {
      if (i>0 && all_fv[k][i] == 0) continue;
      *(t->of) << " " << (i+1) << ":" << all_fv[k][i];
    }
    *(t->of) << endl;
  }
	(void)pthread_mutex_unlock(&mutex_extract_rect);
  delete t;
	return NULL;
}

int main(int argc, char *argv[]) {
	if (argc > 1) {
		N_CYCLES_START = atoi(argv[1]);
		N_CYCLES = N_CYCLES_START + 1;
	}
	if (argc > 2) {
		MODEL_PRE_TRAINED = atoi(argv[2]) > 0 ? true : false;
	}
	if (argc > 3) {
		GENERATE_NEW_TRAINING = atoi(argv[3]) > 0 ? true : false;
	}
	if (argc > 4) {
		EVAL_TEST_SET = atoi(argv[4]) > 0 ? true : false;
	}
	if (argc > 5) {
		EARLY_STOP = atoi(argv[5]) > 0 ? true : false;
	}
  initFilters();
	init();
  string dir = string(dirPrefix);
	vector<int> fNum;
	vector<string> bgFilenames;
	readBgMap((dir+"background_map.txt").c_str(), fNum, bgFilenames);
	
	vector<int> trainingSet, testSet;
	readTrainTestSplit((dir+trainVsTestFile).c_str(), fNum, trainingSet, testSet);
	// readTrainTestSplitCV((dir+trainVsTestCVFile).c_str(), fNum, trainingSet, testSet);
	
	ofstream status;
	status.open(statusFile.c_str(), fstream::out | fstream::app);
        
  double numTotal = 0;
  double precision = 0;
	
	if (GENERATE_NEW_TRAINING) {
		cout << "Generating new training set" << endl;
		status << "Generating new training set" << endl;
		
		ofstream of;
		of.open(trainingFile.c_str());
		// of.open(trainingFile.c_str(), fstream::out | fstream::app);
        
		vector<int> append_rect_fnum;
		vector<Rect> append_rect;
		if (APPEND_RECTS) {
			char buff[200];
			sprintf(buff, "sort -n %s | uniq > %s", appendedRects.c_str(), sortedAppendedRects.c_str());
			system(buff);
			ifstream ap;
			ap.open(sortedAppendedRects.c_str());
			// cout << ap.is_open() << endl;
			// ap.getline(buff,199);
			// cout << buff << endl;
			// float ff;
			// while (ap >> ff) {
					// cout << ff << endl;
			// }
			int fn;
			double isPos;
			Rect r;
			while (ap >> fn >> isPos >> r) {
				r.isPos = isPos;
				append_rect_fnum.push_back(fn);
				append_rect.push_back(r);
				// cout << fn << " " << r << endl;
			}
			ap.close();
		}
        
		//~ vector<string> files = vector<string>();
		//~ getdir(dir,files);
    
		thr_pool_t *pool = thr_pool_create(8, 8, 0, NULL);
    cout << "Training set size: " << trainingSet.size() << endl;
    cout << "append_rect size: " << append_rect.size() << endl;
		// Put in all of the training set
		for (unsigned int nFile = 0; nFile < trainingSet.size(); nFile++) {
			vector<Rect> posRects;
			vector<ScoredRect> rects;
			int fileNum = fNum[trainingSet[nFile]];
			char posBuff[500], negBuff[500];
			// sprintf(posBuff, "%sleft%04d_pos.txt", dir.c_str(), fileNum);
			// sprintf(negBuff, "%sleft%04d_neg.txt", dir.c_str(), fileNum);
			// loadRects(rects,posBuff,negBuff);
			sprintf(posBuff, "%sleft%04d_pos.txt", labelsPrefix, fileNum);
			loadRects(posRects,posBuff);
			unsigned int nPosRects = posRects.size();
			
			for (unsigned int append_rect_pos = 0; append_rect_pos < append_rect_fnum.size()
					; append_rect_pos++)
			{
				if (fileNum == append_rect_fnum[append_rect_pos]) {
					ScoredRect sr;
					sr.rect = append_rect[append_rect_pos];
					sr.score = 0;
					rects.push_back(sr);
				}
			}
			vector<double> result = calcIntersection(rects, posRects, numTotal, precision);
			// if (result.size() == 0)
				// continue;
			for (unsigned int k = 0; k < result.size(); k++) {
				posRects.push_back(rects[k].rect);
			}
            
			// cout << fileNum << ", # rects = " << rects.size() << endl;
			// vector< vector<float> > all_fv = extractTrainingData(dir, fileNum, rects);
			cout << fileNum << ", # rects = " << posRects.size() << endl;
      if (posRects.size() == 0)
        continue;
      thr_pool_queue(pool, work_extract_rect, arg_extract_rect(fileNum, posRects, nPosRects, result, dir, &of));

			// vector< vector<float> > all_fv = extractTrainingData(dir, fileNum, posRects);
			
			// for (unsigned int k=0;k<all_fv.size();k++) {
				// of << (rects[k].isPos?"+1":"-1") << " qid:" << (fileNum+1);
				// of << (k < nPosRects ? 1: result[k-nPosRects]) << " qid:" << (fileNum+1);
				// for (unsigned int i=0;i<all_fv[k].size();i++) {
					// if (i>0 && all_fv[k][i] == 0) continue;
					// of << " " << (i+1) << ":" << all_fv[k][i];
				// }
				// of << endl;
			// }
      
			if (EARLY_STOP && nFile >= 2) {
        break;
			}
		}
    thr_pool_wait(pool);
    thr_pool_destroy(pool);
		of.close();
	}
	for (int nCycle = N_CYCLES_START; nCycle < N_CYCLES; nCycle ++) {
		cout << "Cycle: " << nCycle << ". Training Model..." << endl;
		status << "Cycle: " << nCycle << ". Training Model...";
		
		// Train the model
		char buff[500], modelFile[200];
		sprintf(modelFile, "%s_%04d", modelFilePrefix.c_str(), nCycle);
		if (!(nCycle == N_CYCLES_START && MODEL_PRE_TRAINED)) {
            system("sort -t : -k 2 -n boostedTraining.txt | uniq > sortedBoostedTraining.txt");
			// sprintf(buff, "./svm_learn -z p -t 1 -j 100 %s %s", trainingFile.c_str(), modelFile);
			sprintf(buff, "./svm_rank_learn -c 20 -l 2 %s %s", sortedTrainingFile.c_str(), modelFile);
			system(buff);
      sprintf(buff,"sed 's/SVM-light Version V6.20/SVM-light Version V6.02/' %s > tmp.mod",modelFile,modelFile);
      system(buff);
      sprintf(buff,"mv tmp.mod %s",modelFile);
      system(buff);
		}
		status << "Done" << endl;
		
		// if (EARLY_STOP) {
			// continue;
		// }
		
		MODEL *model = readAndInitModel(modelFile);
		
		if (EVAL_TEST_SET) {
			// Test the model on the test set
			numTotal = 0.000001;
			precision = 0;
			cout << "Testing model on test set..." << endl;
			status << "Testing model on test set..." << endl;
			for (int i=0;i<testSet.size();i++) {
				int fileNum = fNum[testSet[i]];
				// if (EARLY_STOP && (fileNum < 102 || fileNum > 105)) {
					// continue;
				// }
                
				string bgFile = bgFilenames[testSet[i]];
				// Load positive labels
				vector<Rect> rects;
				char posBuff[500];
				// sprintf(posBuff, "%sleft%04d_pos.txt", dir.c_str(), fileNum);
        sprintf(posBuff, "%sleft%04d_pos.txt", labelsPrefix, fileNum);
				loadRects(rects,posBuff);
				
				// Test the model on this test case
        if (fileNum <= 101 || fileNum >= 164) {
					CROP_RECT = cvRect(360, 153, 582, 364);
				} else {
					CROP_RECT = cvRect(400, 0, 1024-400, 768-0);
				}
				vector<ScoredRect> scoredRects = test(dir, fileNum, model, dirPrefix+bgFile);
				
				// Only consider the top rectangle
				vector<ScoredRect> srForTesting; 
				for (int k=0;k<1 && k < scoredRects.size();k++) {
					ScoredRect sr = scoredRects[k];
					for (int p=0;p<4;p++) {
						sr.rect.x[p] += CROP_RECT.x;
						sr.rect.y[p] += CROP_RECT.y;
					}
					srForTesting.push_back(sr);
				}
				// Calculate the test error
				vector<double> result = calcIntersection(srForTesting, rects, numTotal, precision);
				char statusBuff[500];
				sprintf(statusBuff,"Up to test case %d (%d), nCorrect: %d, nTotal: %d, precision: %g, results:",
					i, fileNum, (int)precision, (int)numTotal, (precision/numTotal));
				cout << statusBuff;
				status << statusBuff;
				for (unsigned int k=0;k<result.size();k++) {
					cout << " " << result[k];
					status << " " << result[k];
				}
				cout << endl;
				status << endl;
        
        if (EARLY_STOP && i >= 2) {
          break;
        }
				// cout << "Up to test case " << i << " (" << fileNum << "), precision: " << (precision/numTotal) << endl;
				// status << "Up to test case " << i << " (" << fileNum << "), precision: " << (precision/numTotal) << endl;
			}
			cout << "Test precision: " << (precision/numTotal) << endl;
			status << "Test precision: " << (precision/numTotal) << endl;
		}
		
		// Test the model on the training set
		numTotal = 0.000001;
		precision = 0;
		cout << "Testing model on training set..." << endl;
		status << "Testing model on training set..." << endl;
		vector < vector<ScoredRect> > rectsToAppend;
		rectsToAppend.clear();
		rectsToAppend.resize(trainingSet.size());
		for (int i=0;i<trainingSet.size();i++) {
			int fileNum = fNum[trainingSet[i]];
			string bgFile = bgFilenames[trainingSet[i]];
			// Load positive labels
			vector<Rect> rects;
			char posBuff[500];
			// sprintf(posBuff, "%sleft%04d_pos.txt", dir.c_str(), fileNum);
      sprintf(posBuff, "%sleft%04d_pos.txt", labelsPrefix, fileNum);
			loadRects(rects,posBuff);
      if (rects.size() == 0)
        continue;
			// status << rects.size() << " " << posBuff << endl;
			
			// Test the model on this training case
			if (fileNum <= 101 || fileNum >= 164) {
				CROP_RECT = cvRect(360, 153, 582, 364);
			} else {
				CROP_RECT = cvRect(400, 0, 1024-400, 768-0);
			}
			vector<ScoredRect> scoredRects = test(dir, fileNum, model, dirPrefix+bgFile);
			// status << scoredRects.size() << endl;
			
			// Only consider the top N_RECTS_TO_ADD rectangles
			vector<ScoredRect> srForTesting; 
			for (int k=0;k<N_RECTS_TO_ADD && k < scoredRects.size();k++) {
				ScoredRect sr = scoredRects[k];
				for (int p=0;p<4;p++) {
					sr.rect.x[p] += CROP_RECT.x;
					sr.rect.y[p] += CROP_RECT.y;
				}
				srForTesting.push_back(sr);
			}
			
			// Calculate the test error
			vector<double> result = calcIntersection(srForTesting, rects, numTotal, precision);
			// status << result.size() << endl;
			
			// Add the misranked rectangles to training set
			vector<ScoredRect> rectsAddOn;
			for (unsigned int k=0;k<result.size();k++) {
				if (result[k] < 0.95) { // Add this negative example
					ScoredRect r = srForTesting[k];
					r.score = result[k];
					// r.isPos = false;
					rectsAddOn.push_back(r);
				}
			}
			rectsToAppend[i] = rectsAddOn;
			char statusBuff[500];
			sprintf(statusBuff,"Up to train case %d (%d), nCorrect: %d, nTotal: %d, precision: %g, results:",
				i, fileNum, (int)precision, (int)numTotal, (precision/numTotal));
			cout << statusBuff;
			status << statusBuff;
			for (unsigned int k=0;k<result.size();k++) {
				cout << " " << result[k];
				status << " " << result[k];
			}
			cout << endl;
			status << endl;
			if (EARLY_STOP && i >= 2) {
				break;
			}
			// cout << "Up to train case " << i << " (" << fileNum << "), precision: " << (precision/numTotal) << endl;
			// status << "Up to train case " << i << " (" << fileNum << "), precision: " << (precision/numTotal) << endl;
		}
		cout << "Train precision: " << (precision/numTotal) << endl;
		status << "Train precision: " << (precision/numTotal) << endl;
		
		free_model(model, 1);
        
		// if (EARLY_STOP) {
			// continue;
		// }

		if (ADD_TO_APPENDED_RECTS) {
			// Add new negative examples into training file
			ofstream of;
			of.open(trainingFile.c_str(), fstream::out | fstream::app);
			of << "# " << nCycle << ": train precision = " << (precision/numTotal) << ". Adding Rects..." << endl;
			cout << "# " << nCycle << ": train precision = " << (precision/numTotal) << ". Adding Rects..." << endl;
			ofstream appendRects;
			appendRects.open(appendedRects.c_str(), fstream::out | fstream::app);
      
      thr_pool_t *pool = thr_pool_create(8, 8, 0, NULL);
			
      for (unsigned int nFile = 0; nFile < trainingSet.size(); nFile++) {
				if (rectsToAppend[nFile].size() == 0) 
					continue;
				vector<ScoredRect> rects = rectsToAppend[nFile];
				int fileNum = fNum[trainingSet[nFile]];
				
				vector<Rect> rectsCopy;
				rectsCopy.reserve(rects.size());
				for (unsigned int k=0;k<rects.size();k++) {
					appendRects << fileNum << " " << rects[k] << endl;
					rectsCopy.push_back(rects[k].rect);
				}
				
        char posBuff[200];
        vector<Rect> posRects;
        sprintf(posBuff, "%sleft%04d_pos.txt", labelsPrefix, fileNum);
        loadRects(posRects,posBuff);
        
        vector<double> result = calcIntersection(rects, posRects, numTotal, precision);
        if (result.size() == 0)
          continue;
        
        
				cout << "Adding: " << fileNum << endl;
				status << "Adding: " << fileNum << endl;
        
				thr_pool_queue(pool, work_extract_rect, arg_extract_rect(fileNum, ScoredRect_to_Rect(rects), 0, result, dir, &of));
        
        // vector< vector<float> > all_fv = extractTrainingData(dir, fileNum, rectsCopy);
				
				// for (unsigned int k=0;k<all_fv.size();k++) {
					// of << (rects[k].score) << " qid:" << (fileNum+1);
					// for (unsigned int i=0;i<all_fv[k].size();i++) {
						// if (i>0 && all_fv[k][i] == 0) continue;
						// of << " " << (i+1) << ":" << all_fv[k][i];
					// }
					// of << endl;
				// }
			}
      thr_pool_wait(pool);
      thr_pool_destroy(pool);
			of.close();
			appendRects.close();
		}
	}
	status.close();
	return 0;
}


void readBgMap(const char* filename, vector<int> &fNum, vector<string> &bgFilenames) {
    ifstream in;
    in.open(filename);
    string num, bgf;
    fNum.clear(); bgFilenames.clear();
    while (in >> num >> bgf) {
        fNum.push_back(atoi(num.c_str()));
        bgFilenames.push_back(bgf);
    }
}
