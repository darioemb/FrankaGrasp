
/*
#if !defined(CV_IMPL)
#define CV_IMPL extern "C"
#endif //CV_IMPL

#if defined(__cplusplus)
#include "opencv2/core/internal.hpp"
#endif //__cplusplus
*/

//#include "opencv/cv.h"
//#include "opencv/cxcore.h"
#include "opencv/highgui.h"
#include "histogram.h"

using namespace std;



// Used for threading
//~ vector < ScoredRect > vRect;
//~ pthread_mutex_t mutex;
//~ static int work_count = 0;
//~ typedef struct
//~ {
  //~ CvMat *mat;
  //~ CvRect rect;
  //~ ScoredRect r;
  //~ MODEL *model;
//~ } arg_t;
//~ arg_t *arg(CvMat * mat, CvRect rect, ScoredRect r, MODEL * model)
//~ {
  //~ arg_t *t = new arg_t();
  //~ if (t == NULL)
    //~ printf("MALLOC failed at arg!\n");
  //~ t->mat = mat;
  //~ t->rect = rect;
  //~ t->model = model;
  //~ t->r = r;
  //~ return t;
//~ }
//~ void *work(void *in);


/*
 * RectQueue is the priority queue stored the top k best rects.
 */
//typedef priority_queue<ScoredRect> RectQueue;

