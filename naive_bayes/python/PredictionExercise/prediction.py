#!/usr/bin/env python

from classifier import GNB
import pandas as pd

def convert_to_labels(df):
	new_df = pd.DataFrame(df['states'].values.tolist(),
						  columns=['s', 'd', 's_dot', 'd_dot'])

	new_df['labels'] = df['labels']
	return new_df

def main():
	gnb = GNB()
	train_df = convert_to_labels(pd.read_json('train.json'))
	test_df = convert_to_labels(pd.read_json('test.json'))

	gnb.train(train_df)

	# score = 0
	# for coords, label in zip(X,Y):
	# 	predicted = gnb.predict(coords)
	# 	if predicted == label:
	# 		score += 1
	# fraction_correct = float(score) / len(X)
	# print "You got {} percent correct".format(100 * fraction_correct)

if __name__ == "__main__":
	main()
