#include "util.h"

using namespace std;

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Deep cloning of PointCloud2 message
    * \param input the message in the sensor_msgs::PointCloud2 format
    * \param output the resultant message in the sensor_msgs::PointCloud2 format
    */
void
clone_PointCloud2(const sensor_msgs::PointCloud2 & input,
				  sensor_msgs::PointCloud2 & output)
{
	output.header = input.header;
	output.width = input.width;
	output.height = input.height;
	output.fields.resize(input.fields.size());
	// Copy all fields
	for (size_t d = 0; d < input.fields.size(); ++d)
	{
		output.fields[d].name = input.fields[d].name;
		output.fields[d].offset = input.fields[d].offset;
		output.fields[d].datatype = input.fields[d].datatype;
		output.fields[d].count = input.fields[d].count;
	}
	output.point_step = input.point_step;	// add 4 bytes for index field
	output.row_step = output.point_step * output.width;

	output.is_bigendian = input.is_bigendian;	// @todo ?
	output.is_dense = input.is_dense;

	output.data.resize(input.width * input.height * input.point_step);
	// Copy the data points
	memcpy(&output.data[0], &input.data[0],
		   input.width * input.height * input.point_step);

	return;
}

// Sets the global variables xOffset, yOffset, zOffset, indexOffset;
static void find_offsets(const sensor_msgs::PointCloud2 & pc)
{
	// find x,y,z,index offset
	for (unsigned int i = 0; i < pc.fields.size(); i++)
	{
		if (pc.fields[i].name.compare("z") == 0)
		{
			zOffset = pc.fields[i].offset;
		}
		else if (pc.fields[i].name.compare("x") == 0)
		{
			xOffset = pc.fields[i].offset;
		}
		else if (pc.fields[i].name.compare("y") == 0)
		{
			yOffset = pc.fields[i].offset;
		}
		else if (pc.fields[i].name.compare("index") == 0)
		{
			indexOffset = pc.fields[i].offset;
		}
	}
}


// Edits data in-place.  
// Removes points indexed by ind
static void point_remove_by_index(sensor_msgs::PointCloud2 & pc,
								  pcl::PointIndices ind)
{
	int nPoints = 0, j = 0;
	for (unsigned int i = 0; i < pc.width * pc.height; i++)
	{
		if (j < ind.indices.size() && i == ind.indices[j])
		{
			j++;
		}
		else if (nPoints != i)
		{
			memcpy(&pc.data[nPoints * pc.point_step],
				   &pc.data[i * pc.point_step], pc.point_step);
			nPoints++;
		}
	}
	pc.width = nPoints;
	pc.height = 1;
	pc.is_dense = 0;
	pc.row_step = nPoints * pc.point_step;
	pc.data.resize(nPoints * pc.point_step);
}

// Edits data in-place.  
// Retrieves points indexed by ind
static void point_retrieve_by_index(sensor_msgs::PointCloud2 & pc,
									pcl::PointIndices ind)
{
	printf("indices size: %d\n", ind.indices.size());
	int nPoints = 0;
	for (unsigned int i = 0; i < ind.indices.size(); i++)
	{
		memcpy(&pc.data[i * pc.point_step],
			   &pc.data[ind.indices[i] * pc.point_step], pc.point_step);
	}
	/* 
	   for (unsigned int i=0;i<pc.width*pc.height;i++) { if (i ==
	   ind.indices[nPoints] && nPoints != i) { memcpy(&pc.data[nPoints *
	   pc.point_step], &pc.data[i * pc.point_step], pc.point_step); nPoints
	   ++; if (nPoints >= ind.indices.size()) break; } } */
	pc.width = ind.indices.size();
	pc.height = 1;
	pc.is_dense = 0;
	pc.row_step = pc.width * pc.point_step;
	pc.data.resize(pc.width * pc.point_step);
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Clones and adds an Index field to the point cloud message
    * \param input the message in the sensor_msgs::PointCloud2 format
    * \param output the resultant message in the sensor_msgs::PointCloud2 format
    */
bool
cloneAndAddIndexField(const sensor_msgs::PointCloud2 & input,
					  sensor_msgs::PointCloud2 & output)
{
	if (input.width * input.height < 768 * 1024)
	{
		ROS_ERROR("ERROR: Incoming point cloud (size: %d) is not dense!",
				  input.width * input.height);
		return false;
	}
	output.header = input.header;
	output.width = input.width;
	output.height = input.height;
	output.fields.resize(1 + input.fields.size());	// Extra field to hold
													// index
	// Copy all fields
	for (size_t d = 0; d < input.fields.size(); ++d)
	{
		output.fields[d].name = input.fields[d].name;
		output.fields[d].offset = input.fields[d].offset;
		output.fields[d].datatype = input.fields[d].datatype;
		output.fields[d].count = input.fields[d].count;
	}
	// Set up last field as index
	output.fields[input.fields.size()].name = "index";
	output.fields[input.fields.size()].offset = input.point_step;
	output.fields[input.fields.size()].datatype =
		sensor_msgs::PointField::UINT32;;
	output.fields[input.fields.size()].count = 1;
	output.point_step = input.point_step + 4;	// add 4 bytes for index field
	output.row_step = output.point_step * output.width;

	output.is_bigendian = input.is_bigendian;	// @todo ?
	output.is_dense = input.is_dense;

	output.data.resize(input.width * input.height * output.point_step);
	// Copy the data points
	for (size_t cp = 0; cp < input.width * input.height; ++cp)
	{
		memcpy(&output.data[cp * output.point_step],
			   &input.data[cp * input.point_step], input.point_step);
		// Not sure if this is how to do this.. need to test.
		unsigned int *v =
			(unsigned int *)(&output.
							 data[cp * output.point_step + input.point_step]);
		*v = cp;
	}

	return (true);
}

