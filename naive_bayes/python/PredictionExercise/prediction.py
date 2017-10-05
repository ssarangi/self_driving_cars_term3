#!/usr/bin/env python

from classifier import GNB
import pandas as pd

def convert_to_labels(df):
	new_df = pd.DataFrame(df['states'].values.tolist(),
						  columns=['s', 'd', 's_dot', 'd_dot'])

	new_df['labels'] = df['labels']
	return new_df

def person_test():
	# Create an empty dataframe
	data = pd.DataFrame()

	# Create our target variable
	data['Gender'] = ['male', 'male', 'male', 'male', 'female', 'female', 'female', 'female']

	# Create our feature variables
	data['Height'] = [6, 5.92, 5.58, 5.92, 5, 5.5, 5.42, 5.75]
	data['Weight'] = [180, 190, 170, 165, 100, 150, 130, 150]
	data['Foot_Size'] = [12, 11, 12, 10, 6, 8, 7, 9]

	# Create an empty dataframe
	person = pd.DataFrame()

	# Create some feature values for this single row
	person['Height'] = [6]
	person['Weight'] = [130]
	person['Foot_Size'] = [8]

	gnb = GNB()
	gnb.train(data, "Gender")

	for idx, row in person.iterrows():
		prob_label, prob = gnb.predict(row)
		print("Probability Label: %s --> Probability: %s" % (prob_label, prob))

def main():
	gnb = GNB()
	train_df = convert_to_labels(pd.read_json('train.json'))
	test_df = convert_to_labels(pd.read_json('test.json'))

	gnb.train(train_df, 'labels')

	for idx, row in test_df.iterrows():
		prediction_label, prob = gnb.predict(row)
		print("Predicted: %s Actual: %s" % (prediction_label, row['labels']))

if __name__ == "__main__":
	main()
