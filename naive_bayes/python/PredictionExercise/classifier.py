from pprint import pprint
import numpy as np
import pandas as pd

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
	def __init__(self):
		self.df = None

	def train(self, df : pd.DataFrame, Y_label : str):
		"""
		Trains the classifier with N data points and labels.

		INPUTS
		df - A Dataframe of N observations
		   - 4 column values: s, d, s_dot and d_dot.

		labels - array of N labels
		       - Each label is one of "left", "keep", or "right".
		"""
		self.Y_label = Y_label
		mean_val = df.groupby(Y_label).mean()
		variance = df.groupby(Y_label).var()
		possible_labels = df[Y_label].unique().tolist()

		values = {}
		for label in possible_labels:
			for column in [c for c in df.columns if c != Y_label]:
				values[label + '_' + column + '_means'] = compute_mean(mean_val, label, column)
				values[label + '_' + column + '_variance'] = compute_variance(variance, label, column)

		self.values = values
		self.df = df

	def predict(self, observation : pd.Series):
		"""
		Once trained, this method is called and expected to return
		a predicted behavior for the given observation.

		INPUTS

		observation - a pandas row s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		unique_labels = self.df[self.Y_label].unique().tolist()

		max_prob = -1
		max_prob_label = ''
		for label in unique_labels:
			prob_label = self.df[self.Y_label][self.df[self.Y_label] == label].count() / self.df[self.Y_label].count()

			current_prob = 1
			for column, column_value in observation.iteritems():
				if column != self.Y_label:
					current_prob *= p_x_given_y(
						column_value, self.values[label + '_' + column + '_means'], self.values[label + '_' + column + '_variance'])

			current_prob *= prob_label

			if current_prob > max_prob:
				max_prob = current_prob
				max_prob_label  = label

		return max_prob_label, max_prob