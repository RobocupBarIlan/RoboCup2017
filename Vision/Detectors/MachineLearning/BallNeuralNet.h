/*
*BallNeuralNet.h
*
*This class is responsible for holding a sigmoid neural network with 1 hidden layer. 
* It supports querying the neural network (feedforwarding) - by calling the query(input) function.
* Loading the neural network's data is done using the setWeightsInputHidden(path) and  setWeightsHiddenOutput(path) methods *in constructor* !
 */


#ifndef NULL
#define NULL   ((void *) 0)
#endif

#ifndef VISION_DETECTORS_BallNeuralNet_H_
#define VISION_DETECTORS_BallNeuralNet_H_

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#define INPUT_SIZE_BALL_NEURAL_NET 3500 //The size of an input image is 35X70=2450.
#define HIDDEN_LAYER_SIZE_BALL_NEURAL_NET 25 //Number of activation units in hidden layer.
#define OUTPUT_SIZE_BALL_NEURAL_NET 1 //This neural network will only have 1 class for classification.

using namespace std;


class BallNeuralNet {
private:
	double m_weights_input_hidden[HIDDEN_LAYER_SIZE_BALL_NEURAL_NET][INPUT_SIZE_BALL_NEURAL_NET+1]; //+1 for bias.
	double m_weights_hidden_output[OUTPUT_SIZE_BALL_NEURAL_NET][HIDDEN_LAYER_SIZE_BALL_NEURAL_NET+1]; //+1 for bias.
	int m_hidden_layer_size; //Determines how many activation units there are in hidden layer.
	int m_input_size; //Determines how many inputs there are to the neural network.
	int m_output_size; //Determines how many outputs there are to the neural network.
	void setWeightsInputHidden(string path); //This method sets the m_weights_input_hidden 2D array.
	void setWeightsHiddenOutput(string path); //This method sets the m_weights_hidden_output array.
	static double Sigmoid(double x); //Calculates the sigmoid function of x.
public:
	BallNeuralNet(int input_size,int hidden_layer_size,int output_size,string path_weights_input_hidden,string path_weights_hidden_output);
	virtual ~BallNeuralNet();
	double query(double input[INPUT_SIZE_BALL_NEURAL_NET],int size); //This method gets an input to the neural network (a double array of size INPUT_SIZE) and returns the query result of the neural network.

};

#endif /* VISION_DETECTORS_BallNeuralNet_H_ */
