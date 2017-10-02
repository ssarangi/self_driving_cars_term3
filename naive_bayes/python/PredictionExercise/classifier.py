from pprint import pprint
import numpy as np

def compute_mean(mean_val, label_name, column_name):
	mean_v = mean_val[column_name][mean_val.index == label_name].values[0]
	return mean_v

def compute_variance(variance, label_name, column_name):
	var = variance[column_name][variance.index == label_name].values[0]
	return var

# Create a function that calculates p(x | y):
def p_x_given_y(x, mean_y, variance_y):
	# Input the arguments into a probability density function
    p = 1/(np.sqrt(2*np.pi*variance_y)) * np.exp((-(x-mean_y)**2)/(2*variance_y))

	# return p
    return p

class GNB(object):
	def train(self, df):
		"""
		Trains the classifier with N data points and labels.

		INPUTS
		df - A Dataframe of N observations
		   - 4 column values: s, d, s_dot and d_dot.

		labels - array of N labels
		       - Each label is one of "left", "keep", or "right".
		"""
		mean_val = df.groupby('labels').mean()
		variance = df.groupby('labels').var()
		possible_labels = df['labels'].unique().tolist()

		values = {}
		for label in possible_labels:
			for column in [c for c in df.columns if c != 'labels']:
				values[label + '_' + column + '_means'] = compute_mean(mean_val, label, column)
				values[label + '_' + column + '_variance'] = compute_variance(variance, label, column)

		pprint(values)

	def predict(self, observation):
		"""
		Once trained, this method is called and expected to return
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
		return self.possible_labels[1]
