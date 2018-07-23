#include "util.h"

using namespace std;

// Tries to find the set of points in pc1 but not in pc2.
// pc2 must be a strict subset of pc1
static void point_cloud_difference(sensor_msgs::PointCloud2 & pc1,
								   sensor_msgs::PointCloud2 & pc2,
								   sensor_msgs::PointCloud2 & output)
{
	output.header = pc1.header;
	output.width = pc1.width - pc2.width;
	output.height = pc1.height;
	output.fields.resize(pc1.fields.size());
	// Copy all fields
	for (size_t d = 0; d < pc1.fields.size(); ++d)
	{
		output.fields[d].name = pc1.fields[d].name;
		output.fields[d].offset = pc1.fields[d].offset;
		output.fields[d].datatype = pc1.fields[d].datatype;
		output.fields[d].count = pc1.fields[d].count;
	}
	output.point_step = pc1.point_step;	// add 4 bytes for index field
	output.row_step = output.point_step * output.width;

	output.is_bigendian = pc1.is_bigendian;	// @todo ?
	output.is_dense = pc1.is_dense;

	output.data.resize(output.width * output.height * output.point_step);
	find_offsets(pc1);

	unsigned int nPoints = 0;
	unsigned int a = 0, b = 0;
	while (nPoints < output.width)
	{
		if (a >= pc1.width)
		{
			printf("ERROR: Difference finding failed\n");
			return;
		}
		if (b >= pc2.width)
		{						// done with pc2.  copy rest of pc1
			// printf("DONE with pc2\n");
			// printf("%d,%d\n",pc1.width-a,output.width-nPoints);
			memcpy(&output.data[nPoints * output.point_step],
				   &pc1.data[a * pc1.point_step],
				   (pc1.width - a) * pc1.point_step);
			nPoints += (pc1.width - a);
			a = pc1.width;
			// printf("DONE with pc1\n");
			return;
		}
		else
		{
			unsigned int *i1 =
				(unsigned int *)(&pc1.data[a * pc1.point_step + indexOffset]);
			unsigned int *i2 =
				(unsigned int *)(&pc2.data[b * pc2.point_step + indexOffset]);
			if (*i1 == *i2)
			{
				a++;
				b++;
			}
			else
			{
				memcpy(&output.data[nPoints * output.point_step],
					   &pc1.data[a * pc1.point_step], pc1.point_step);
				a++;
				nPoints++;
			}
		}
	}
	/* while (!(a>=pc1.width || b>=pc2.width)) { unsigned int* i1 = (unsigned
	   int*)(&pc1.data[a*pc1.point_step+indexOffset]); unsigned int* i2 =
	   (unsigned int*)(&pc2.data[b*pc2.point_step+indexOffset]); if (*i1 ==
	   *i2) { a++; b++; } else { printf("ERROR: Points remain that are
	   different!"); break; a++; nPoints++; } } */
}

// Edits data in-place.
// Removes statistical outliers
static void remove_statistical_outliers(sensor_msgs::PointCloud2 & pc)
{
	pcl::StatisticalOutlierRemoval < sensor_msgs::PointCloud2 > sor;
	// write_point_cloud("pc1.txt",pc);
	sor.setInputCloud(boost::make_shared < sensor_msgs::PointCloud2 > (pc));
	sor.setMeanK(outlier_mean_k);
	sor.setStddevMulThresh(outlier_StddevMulThresh);
	sensor_msgs::PointCloud2 pc2;
	sor.filter(pc2);
	// printf("Done filter\n");
	// write_point_cloud("pc2.txt",pc);
	sensor_msgs::PointCloud2 pc3;
	point_cloud_difference(pc, pc2, pc3);
	// printf("Done difference\n");
	// write_point_cloud("pc3.txt",pc3);
	chatter_pub3.publish(pc3);
	// printf("Done publish\n");
	clone_PointCloud2(pc2, pc);
	// printf("Done clone\n");
	// pc = pc2;
	// exit (0);
}

