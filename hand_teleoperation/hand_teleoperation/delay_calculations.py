import pandas as pd
import csv

# USING MICROROS
df = pd.read_csv('delay_data.csv')
average_ros = df['delay_ms'].mean()
median_ros = df['delay_ms'].median()
min_ros = df['delay_ms'].min()
max_ros = df['delay_ms'].max()
std_ros = df['delay_ms'].std()

MR = ['---- MICROROS DELAY CALCULATIONS ----\n', f'Average delay using microros {average_ros} ms \n', f'Median delay using microros {median_ros} ms \n',
     f'Minimum delay using microros {min_ros} ms \n', f'Maximun delay using microros {max_ros} ms \n',
     f'Standard deviation delay using microros {std_ros} ms \n']

f1 = open("processed_data.txt", 'w')

f1.writelines(MR)
