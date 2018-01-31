'''
***************************************************
File: assignment0.py

CSCI 5822 assignment 0 code

***************************************************
'''

from __future__ import division
from pandas import DataFrame
import numpy as np

__author__="Jeremy Muesing"
__email__="jeremy.muesing@colorado.edu"
__version__="1.0.0"

# load data
text_file=open("titanic.txt",'r')
lines=text_file.read().split('\n')
i=0
for line in lines:
    items=line.split()
    if items:
        new_data=DataFrame(np.array([[items[0],items[1],items[2],items[3]]]),index=[str(i)],columns=('class','age','sex','status'))
        i=i+1
    if i==1:
        data=new_data
    else:
        data=data.append(new_data)

# The following for loop executes both task 0 and task 1
task0_df=DataFrame()
task1_df=DataFrame()
task1_classify=DataFrame()
for gender in ['male','female']:
    for age in ['child','adult']:
        for rank in ['1st','2nd','3rd','crew']:
            task1_new_df=data[(data['sex']==gender) & (data['age']==age) & (data['class']==rank)]
            if len(task1_new_df)!=0:
                prob_live=len(task1_new_df[(task1_new_df['status']=='yes')])/len(task1_new_df)
                prob_die=len(task1_new_df[(task1_new_df['status']=='no')])/len(task1_new_df)
            else:
                prob_live=None
                prob_die=None
                # creating the task 1 table
            task1_df=task1_df.append(DataFrame(np.array([[prob_live,prob_die]]),index=[str(gender+age+rank)],columns=('survival','death')))
            if prob_live<prob_die:
                outcome_class='die'
            elif prob_live>prob_die:
                outcome_class='live'
            elif not prob_live:
                outcome_class=None
            # creating the task 1 classification
            task1_classify=task1_classify.append(DataFrame(np.array([[outcome_class]]),index=[str(gender+age+rank)],columns=['classification']))
            for outcome in ['yes','no']:
                # creating the task 0 table
                new_df=data[(data['sex']==gender) & (data['age']==age) & (data['class']==rank) & (data['status']==outcome)]
                prob=len(new_df)/len(data)
                task0_df=task0_df.append(DataFrame(np.array([[prob]]),index=[str(gender+age+rank+outcome)]))

# making the 6 one dimentional tables for part 1 of task 2
task2_gender_live=DataFrame()
task2_gender_die=DataFrame()
task2_class_live=DataFrame()
task2_class_die=DataFrame()
task2_age_live=DataFrame()
task2_age_die=DataFrame()
task2_total_outcome=[]
for outcome in ['yes','no']:
    task2_outcome_df=data[data['status']==outcome]
    task2_total_outcome.append(len(task2_outcome_df)/len(data))
    for gender in ['male','female']:
        task2_gender_prob=len(task2_outcome_df[task2_outcome_df['sex']==gender])/len(task2_outcome_df)
        if outcome=='yes':
            task2_gender_live=task2_gender_live.append(DataFrame(np.array([[task2_gender_prob]]),index=[gender],columns=['live']))
        elif outcome=='no':
            task2_gender_die=task2_gender_die.append(DataFrame(np.array([[task2_gender_prob]]),index=[gender],columns=['die']))
    for rank in ['1st','2nd','3rd','crew']:
        task2_class_prob=len(task2_outcome_df[task2_outcome_df['class']==rank])/len(task2_outcome_df)
        if outcome=='yes':
            task2_class_live=task2_class_live.append(DataFrame(np.array([[task2_class_prob]]),index=[rank],columns=['live']))
        elif outcome=='no':
            task2_class_die=task2_class_die.append(DataFrame(np.array([[task2_class_prob]]),index=[rank],columns=['die']))
    for age in ['child','adult']:
        task2_age_prob=len(task2_outcome_df[task2_outcome_df['age']==age])/len(task2_outcome_df)
        if outcome=='yes':
            task2_age_live=task2_age_live.append(DataFrame(np.array([[task2_age_prob]]),index=[age],columns=['live']))
        elif outcome=='no':
            task2_age_die=task2_age_die.append(DataFrame(np.array([[task2_age_prob]]),index=[age],columns=['die']))

# printing answers
print "-----Pr(Y|Gender,Age,Class)-----"
print task1_df
print "-----Classification based off features-----"
print task1_classify
print "-----Pr(X|Y)-----"
print task2_gender_live
print task2_gender_die
print task2_class_live
print task2_class_die
print task2_age_live
print task2_age_die
print "-----Unconditional Probabilities-----"
print "Pr(survival)=%f" % task2_total_outcome[0]
print "Pr(death)=%f" % task2_total_outcome[1]
print "-----Naive Bayes Pr(Death|Gender,Age,Class) + Classification-----"
# executing naive bayes, classifying, and printing the answers
for gender in ['male','female']:
    for age in ['child','adult']:
        for rank in ['1st','2nd','3rd','crew']:
            numerator_die=task2_gender_die.loc[gender].values[0]*task2_age_die.loc[age].values[0]*task2_class_die.loc[rank].values[0]*task2_total_outcome[1]
            numerator_live=task2_gender_live.loc[gender].values[0]*task2_age_live.loc[age].values[0]*task2_class_live.loc[rank].values[0]*task2_total_outcome[0]
            normalize=numerator_die+numerator_live
            if (numerator_die/normalize)>(numerator_live/normalize):
                print "Pr(death|%s,%s,%s)=%f Pr(survival)=%f %s" % (gender,age,rank,numerator_die/normalize,numerator_live/normalize,'die')
            else:
                print "Pr(death|%s,%s,%s)=%f Pr(survival)=%f %s" % (gender,age,rank,numerator_die/normalize,numerator_live/normalize,'live')
