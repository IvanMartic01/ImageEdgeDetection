#include <iostream>
#include <stdlib.h>
#include "BitmapRawConverter.h"
#include "tbb/task_group.h"
#include "tbb/tick_count.h"

/*
				j=column
		-------------------------------------------------------------------------
		|                 |                 |                 |                 |
i=row	|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		-------------------------------------------------------------------------
		|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		-------------------------------------------------------------------------
		|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		-------------------------------------------------------------------------
		|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		-------------------------------------------------------------------------
		|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		|                 |                 |                 |                 |
		-------------------------------------------------------------------------
*/

#define __ARG_NUM__				6
#define FILTER_SIZE				3
#define THRESHOLD				128
#define AREA 0 // area of search around matrix (edge detection), 0 standard level
using namespace tbb;
using namespace std;

// Prewitt operators 3x3

int filterHor[FILTER_SIZE * FILTER_SIZE] = {
	-1, 0, 1,
	-1, 0, 1,
	-1, 0, 1
};
int filterVer[FILTER_SIZE * FILTER_SIZE] = {
	-1, -1, -1,
	0, 0, 0,
	1, 1, 1
};

// Prewitt operators 5x5

//int filterHor[FILTER_SIZE * FILTER_SIZE] = {
//9, 9, 9, 9, 9,
//9, 5, 5, 5, 9,
//-7, -3, 0, -3, -7,
//-7, -3, -3, -3, -7,
//-7, -7, -7, -7, -7
//};
//int filterVer[FILTER_SIZE * FILTER_SIZE] = {
//	9, 9, -7, -7, -7,
//	9, 5, -3, -3, -7,
//	9, 5, 0, -3, -7,
//	9, 5, -3, -3, -7,
//	9, 9, -7, -7, -7
//};

// Prewitt operators 7x7
//
//int filterHor[FILTER_SIZE * FILTER_SIZE] = {
//	-1, -1, -1, 0, 1, 1, 1,
//	-1, -1, -1, 0, 1, 1, 1,
//	-1, -1, -1, 0, 1, 1, 1,
//	-1,-1, -1, 0, 1, 1, 1,
//	-1,-1, -1, 0, 1, 1, 1,
//	-1,-1, -1, 0, 1, 1, 1,
//	-1,-1, -1, 0, 1, 1, 1
//};
//int filterVer[FILTER_SIZE * FILTER_SIZE] = {
//	-1, -1, -1, -1, -1, -1, -1,
//	-1, -1, -1, -1, -1, -1, -1,
//	-1, -1, -1, -1, -1, -1, -1,
//	0, 0, 0, 0, 0, 0, 0,
//	1, 1, 1, 1, 1, 1, 1,
//	1, 1, 1, 1, 1, 1, 1,
//	1, 1, 1, 1, 1, 1, 1
//};

/**
* @brief Serial version of edge detection algorithm implementation using Prewitt operator
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/

int remove_color(int point_value) {
	if (point_value <= THRESHOLD) {
		return 0;
	}

	return 255;
}

void new_row_prewitt(int& start_index_i, int& start_index_j, int& end_index_j) {
	start_index_i++;
	start_index_j = 0;
	end_index_j = FILTER_SIZE - 1;
}

void new_column(int& start_index_j) {
	start_index_j++;
}

int calculateG(int start_index_i, int start_index_j, int* inBuffer, int width) {
	int initial_index_j = start_index_j;
	// Initial value of Horizontal component
	int Gx = 0;
	// Initial value of Vertical component
	int Gy = 0;

	// Calculating Gx and Gy
	for (int i = 0; i < FILTER_SIZE; i++) {
		for (int j = 0; j < FILTER_SIZE; j++) {
			Gx += filterHor[(i * FILTER_SIZE) + j] * inBuffer[(start_index_i * width) + start_index_j];
			Gy += filterVer[(i * FILTER_SIZE) + j] * inBuffer[(start_index_i * width) + start_index_j];
			start_index_j++;
		}
		start_index_j = initial_index_j;
		start_index_i++;
	}

	// G = |Gx| + |Gy|
	int G = abs(Gx) + abs(Gy);
	// G is scaled to be black or white
	G = remove_color(G);
	return G;
}

void calculate_row_prewitt(int* inBuffer, int* outBuffer, int start_index_i, int end_index_i, int start_index_j, int end_index_j, int width) {
	for (; end_index_j < width; end_index_j++) {
		// index of point we are changing
		int index_i_of_pixel_for_change = (start_index_i + end_index_i) / 2;
		int index_j_of_pixel_for_change = (start_index_j + end_index_j) / 2;

		// G = |Gx| + |Gy| => scaled G
		int G = calculateG(start_index_i, start_index_j, inBuffer, width);

		// Scaled value is written in point
		outBuffer[(index_i_of_pixel_for_change * width) + index_j_of_pixel_for_change] = G;

		new_column(start_index_j);
	}
}

void calculate_serial_prewitt(int* inBuffer, int* outBuffer, int width, int height)
{
	// initial index values
	int start_index_i = 0;
	int end_index_i = FILTER_SIZE - 1;
	int start_index_j = 0;
	int end_index_j = FILTER_SIZE - 1;

	for (; end_index_i < height; end_index_i++) {
		calculate_row_prewitt(inBuffer, outBuffer, start_index_i, end_index_i, start_index_j, end_index_j, width);
		new_row_prewitt(start_index_i, start_index_j, end_index_j);
	}
}

/*
* @brief Parallel version of edge detection algorithm implementation using Prewitt operator
*
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/

void calculate_parallel_prewitt(int* inBuffer, int* outBuffer, int width, int height)
{
	// initial index values
	int start_index_i = 0;
	int end_index_i = FILTER_SIZE - 1;
	int start_index_j = 0;
	int end_index_j = FILTER_SIZE - 1;

	task_group g;
	for (; end_index_i < height; end_index_i++) {
		// Each row is done parallel
		g.run([=] {
			calculate_row_prewitt(inBuffer, outBuffer, start_index_i, end_index_i, start_index_j, end_index_j, width);
			});
		new_row_prewitt(start_index_i, start_index_j, end_index_j);
	}
	g.wait();
}

/**
* @brief Serial version of edge detection algorithm
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/

// Making picture to be black and white
void make_image_black_white(int* inBuffer, int width, int height) {
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (inBuffer[(i * width) + j] <= THRESHOLD) {
				inBuffer[(i * width) + j] = 0;
			}
			else {
				inBuffer[(i * width) + j] = 1;
			}
		}
	}
}

void copyBuffer(int* inBuffer, int* outBuffer, int width, int height) {
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			outBuffer[(i * width) + j] = inBuffer[(i * width) + j];
		}
	}
}

void new_row_edge_detection(int& start_index_i, int& start_index_j, int& end_index_j) {
	start_index_i++;
	start_index_j = 0 + AREA;
	end_index_j = 2 + AREA;
}

// rarrange point_value to be 0 or 255
int return_result_in_range(int point_result) {
	if (point_result == 0) {
		return 0;
	}

	return 255;
}

int calculate_PO(int* inBuffer, int index_i_of_pixel_for_change, int index_j_of_pixel_for_change, int width) {
	// If there is no point with a value of 1 around our point;
	int P = 0;
	// If there is no point with a value of 0 around our point
	int O = 1;

	for (int i = index_i_of_pixel_for_change - (1 + AREA); i < (index_i_of_pixel_for_change + 2 + AREA); i++) {
		for (int j = index_j_of_pixel_for_change - (1 + AREA); j < (index_j_of_pixel_for_change + 2 + AREA); j++) {
			if (i == index_i_of_pixel_for_change && j == index_j_of_pixel_for_change) {
				continue;
			}

			// If there is a point with a value of 1 around our point
			if ((inBuffer[(i * width) + j]) == 1) {
				P = 1;
			}
			// If there is a point with a value of 0 around our point
			if ((inBuffer[(i * width) + j]) == 0) {
				O = 0;
			}
		}
	}

	int point_result = abs(P - O);
	point_result = return_result_in_range(point_result);
	return point_result;
}

void calculate_row_edge_detection(int* inBuffer, int* outBuffer, int start_index_i, int end_index_i, int start_index_j, int end_index_j, int width) {
	for (; end_index_j < width - AREA; end_index_j++) {
		int index_i_of_pixel_for_change = (start_index_i + end_index_i) / 2;
		int index_j_of_pixel_for_change = (start_index_j + end_index_j) / 2;

		int point_result = calculate_PO(inBuffer, index_i_of_pixel_for_change, index_j_of_pixel_for_change, width);
		outBuffer[(index_i_of_pixel_for_change * width) + index_j_of_pixel_for_change] = point_result;
		new_column(start_index_j);
	}
}

void calculate_serial_edge_detection(int* inBuffer, int* outBuffer, int width, int height)	//TODO obrisati
{
	make_image_black_white(inBuffer, width, height);
	//initial index values
	int start_index_i = 0 + AREA;
	int end_index_i = 2 + AREA;
	int start_index_j = 0 + AREA;
	int end_index_j = 2 + AREA;

	for (; end_index_i < height - AREA; end_index_i++) {
		calculate_row_edge_detection(inBuffer, outBuffer, start_index_i, end_index_i, start_index_j, end_index_j, width);
		new_row_edge_detection(start_index_i, start_index_j, end_index_j);
	}
}

/**
* @brief Parallel version of edge detection algorithm
*
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/
void calculate_parallel_edge_detection(int* inBuffer, int* outBuffer, int width, int height)
{
	make_image_black_white(inBuffer, width, height);
	//initial index values
	int start_index_i = 0 + AREA;
	int end_index_i = 2 + AREA;
	int start_index_j = 0 + AREA;
	int end_index_j = 2 + AREA;

	task_group g;
	for (; end_index_i < height - AREA; end_index_i++) {
		g.run([=] {
			calculate_row_edge_detection(inBuffer, outBuffer, start_index_i, end_index_i, start_index_j, end_index_j, width);
			});
		new_row_edge_detection(start_index_i, start_index_j, end_index_j);
	}
	g.wait();
}

/**
* @brief Function for running test.
*
* @param testNr test identification, 1: for serial version, 2: for parallel version
* @param ioFile input/output file, firstly it's holding buffer from input image and than to hold filtered data
* @param outFileName output file name
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/

void run_test_nr(int testNr, BitmapRawConverter* ioFile, char* outFileName, int* outBuffer, unsigned int width, unsigned int height)
{
	// TODO: start measure
	tick_count t0 = tick_count::now();
	switch (testNr)
	{
	case 1:
		cout << "Running serial version of edge detection using Prewitt operator" << endl;
		calculate_serial_prewitt(ioFile->getBuffer(), outBuffer, width, height);
		break;
	case 2:
		cout << "Running parallel version of edge detection using Prewitt operator" << endl;
		calculate_parallel_prewitt(ioFile->getBuffer(), outBuffer, width, height);
		break;
	case 3:
		cout << "Running serial version of edge detection" << endl;
		calculate_serial_edge_detection(ioFile->getBuffer(), outBuffer, width, height);
		break;
	case 4:
		cout << "Running parallel version of edge detection" << endl;
		calculate_parallel_edge_detection(ioFile->getBuffer(), outBuffer, width, height);
		break;
	default:
		cout << "ERROR: invalid test case, must be 1, 2, 3 or 4!";
		break;
	}
	// TODO: end measure and display time
	tick_count t1 = tick_count::now();
	printf("work took %g seconds\n", (t1 - t0).seconds());

	ioFile->setBuffer(outBuffer);
	ioFile->pixelsToBitmap(outFileName);
}

/**
* @brief Print program usage.
*/
void usage()
{
	cout << "\n\ERROR: call program like: " << endl << endl;
	cout << "ProjekatPP.exe";
	cout << " input.bmp";
	cout << " outputSerialPrewitt.bmp";
	cout << " outputParallelPrewitt.bmp";
	cout << " outputSerialEdge.bmp";
	cout << " outputParallelEdge.bmp" << endl << endl;
}

int main(int argc, char* argv[])
{
	if (argc != __ARG_NUM__)
	{
		usage();
		return 0;
	}

	BitmapRawConverter inputFile(argv[1]);
	BitmapRawConverter outputFileSerialPrewitt(argv[1]);
	BitmapRawConverter outputFileParallelPrewitt(argv[1]);
	BitmapRawConverter outputFileSerialEdge(argv[1]);
	BitmapRawConverter outputFileParallelEdge(argv[1]);

	unsigned int width, height;

	int test;

	width = inputFile.getWidth();
	height = inputFile.getHeight();

	int* outBufferSerialPrewitt = new int[width * height];
	int* outBufferParallelPrewitt = new int[width * height];

	memset(outBufferSerialPrewitt, 0x0, width * height * sizeof(int));
	memset(outBufferParallelPrewitt, 0x0, width * height * sizeof(int));

	int* outBufferSerialEdge = new int[width * height];
	int* outBufferParallelEdge = new int[width * height];

	memset(outBufferSerialEdge, 0x0, width * height * sizeof(int));
	memset(outBufferParallelEdge, 0x0, width * height * sizeof(int));

	// serial version Prewitt
	run_test_nr(1, &outputFileSerialPrewitt, argv[2], outBufferSerialPrewitt, width, height);

	// parallel version Prewitt
	run_test_nr(2, &outputFileParallelPrewitt, argv[3], outBufferParallelPrewitt, width, height);

	// serial version special
	run_test_nr(3, &outputFileSerialEdge, argv[4], outBufferSerialEdge, width, height);

	// parallel version special
	run_test_nr(4, &outputFileParallelEdge, argv[5], outBufferParallelEdge, width, height);

	// verification
	cout << "Verification: ";
	test = memcmp(outBufferSerialPrewitt, outBufferParallelPrewitt, width * height * sizeof(int));

	if (test != 0)
	{
		cout << "Prewitt FAIL!" << endl;
	}
	else
	{
		cout << "Prewitt PASS." << endl;
	}

	test = memcmp(outBufferSerialEdge, outBufferParallelEdge, width * height * sizeof(int));

	if (test != 0)
	{
		cout << "Edge detection FAIL!" << endl;
	}
	else
	{
		cout << "Edge detection PASS." << endl;
	}

	// clean up
	delete outBufferSerialPrewitt;
	delete outBufferParallelPrewitt;

	delete outBufferSerialEdge;
	delete outBufferParallelEdge;

	return 0;
}