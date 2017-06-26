/*
 * BallNeuralNet.cpp
 *
 */

#include "BallNeuralNet.h"


BallNeuralNet::BallNeuralNet(int input_size,int hidden_layer_size,int output_size,string path_weights_input_hidden,string path_weights_hidden_output) {
	this->m_input_size=input_size;
	this->m_hidden_layer_size=hidden_layer_size;
	this->m_output_size=output_size;
//	//Allocate memory space for neural network's weights:
//	m_weights_input_hidden = new double*[m_hidden_layer_size];
//	for (int i = 0; i < m_hidden_layer_size; i++)
//		m_weights_input_hidden[i] = new double[m_input_size+1]; //+1 for the bias unit.
//
//	m_weights_hidden_output = new double*[m_output_size];
//	for (int i = 0; i < m_output_size; i++)
//		m_weights_hidden_output[i] = new double[m_hidden_layer_size+1]; //+1 for the bias unit.
	
	//Load the data of the neural net:
	this->setWeightsInputHidden(path_weights_input_hidden);
	this->setWeightsHiddenOutput(path_weights_hidden_output);
}

BallNeuralNet::~BallNeuralNet(){
//	//Deallocate dynamically allocated memory:
//	for(int i = 0; i < m_hidden_layer_size; i++)
//	{
//		delete [] m_weights_input_hidden[i];
//	}
//	delete[] m_weights_input_hidden;
//
//
//	for(int i = 0; i < m_output_size; i++)
//	{
//		delete [] m_weights_hidden_output[i];
//	}
//	delete[] m_weights_hidden_output;
}



double BallNeuralNet::Sigmoid(double x)
{
	return 1/(1+exp(-1*x));
}

void BallNeuralNet::setWeightsInputHidden(string path)
{
	ifstream in(path); //Initialize an input stream.
	if (!in) {
		cout << "Cannot open file containing the input-hidden weights for neural network.\n";
		return;
	}
	for (int i = 0; i < m_hidden_layer_size; i++) {
	for (int j = 0; j < m_input_size+1; j++) {
			in >> m_weights_input_hidden[i][j]; //Read the weights into the 2D array m_weights_input_hidden.
			//cout<<"m_weights_input_hidden[i][j]"<<i<<","<<j<<":"<< m_weights_input_hidden[i][j]<<endl;
		}
	}
	
	in.close();
}

void BallNeuralNet::setWeightsHiddenOutput(string path)
{

	ifstream in(path); //Initialize an input stream.
	if (!in) {
		cout << "Cannot open file containing the hidden-output weights for neural network.\n";
		return;
	}
	for (int i = 0; i < m_output_size; i++) {
	for (int j = 0; j < m_hidden_layer_size+1; j++) {
			in >> m_weights_hidden_output[i][j]; //Read the weights into the array m_weights_hidden_output.
		}
	}
	
	in.close();
}

/*
This method gets an input to the (sigmoid) neural network and returns the query result of the neural network.
*/
double BallNeuralNet::query(double input[INPUT_SIZE_BALL_NEURAL_NET],int size) //Input must be of size m_input_size. The caller is responsible for deallocating the input array.
{

	if(size==m_input_size)
	{
		try
		{
			//Set the input to be consisted of only 1's(where pixel is positive) and 0's (where pixel is 0)  -for the neural network to work.
			for(int i=0;i<m_input_size;i++)
			{
				//cout<<"input["<<i<<"]:"<<input[i]<<endl;
				if(input[i]>0)
				{
					input[i]=1;
				}
			}

			double hidden_inputs[m_hidden_layer_size];
			double hidden_outputs[m_hidden_layer_size];
		//	cout<<"im here"<<endl;
			for(int i=0;i<m_hidden_layer_size;i++) //Init to all cells with the bias of each activation unit. Then - calculate the input to each of the activation units of the hidden layer:
			{
				hidden_inputs[i]=m_weights_input_hidden[i][0]; //Init to all cells with the bias of each activation unit
				//cout<<"m_weights_input_hidden[i][0]"<<m_weights_input_hidden[i][0]<<endl;
				for (int j=0;j<m_input_size;j++)
				{
					//cout<<"j"<<j<<endl;
					hidden_inputs[i]=hidden_inputs[i]+m_weights_input_hidden[i][j+1]*input[j];
				}
				//cout<<"hidden_inputs["<<i<<"]:"<<hidden_inputs[i]<<endl;
			}
			//Apply sigmoid to all of the calculated values:
			for(int i=0;i<m_hidden_layer_size;i++)
			{
				hidden_outputs[i]=BallNeuralNet::Sigmoid(hidden_inputs[i]);
				//cout<<"hidden_outputs[i]"<<hidden_outputs[i]<<endl;
			}
			cout<<endl;
			//Calculate the input to the activation unit of the output layer:
			double output_layer_input,output_layer_output;
			output_layer_input=m_weights_hidden_output[0][0]; //Init to the bias value first.
			for(int i=0;i<m_hidden_layer_size;i++)
			{
				output_layer_input=output_layer_input+m_weights_hidden_output[0][i+1]*hidden_outputs[i];
			}
			//Apply sigmoid to the output_layer_input to get the output_layer_output:
			output_layer_output=BallNeuralNet::Sigmoid(output_layer_input);
		//	cout<<"output_layer_output:"<<output_layer_output<<endl;
			return output_layer_output;
		}catch(std::exception e)
		{
			cout<<e.what();
			return -1;
		}
	}
	else
	{
		cout<<"Size of input to neural network is wrong. please check the size of the input for BallNeuralNet object."<<endl;
	}
	return -1;

}


