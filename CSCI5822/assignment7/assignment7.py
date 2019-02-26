from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from sklearn.feature_extraction.text import CountVectorizer
from sklearn.decomposition import LatentDirichletAllocation
from sklearn.metrics.pairwise import cosine_similarity

# taken from an example on LDA
def print_top_words(model, feature_names, n_top_words):
    for topic_idx, topic in enumerate(model.components_):
        message = "Topic #%d: " % topic_idx
        message += " ".join([feature_names[i]
                             for i in topic.argsort()[:-n_top_words - 1:-1]])
        print message

# part I
letters=['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T']
t=np.random.dirichlet([0.1]*3,200)
topic_words=[[],[],[]]
documents=np.zeros((200,50,2))
for i in range(documents.shape[0]):
    w=np.random.dirichlet([0.01]*20,documents.shape[1])
    for j in range(len(w)):
        word_sample=np.random.multinomial(1,w[j])
        t=np.random.dirichlet([0.1]*3)
        topic_sample=np.random.multinomial(1,t)
        #  print letters[word_sample.tolist().index(1)],topic_sample.tolist().index(1)
        word=word_sample.tolist().index(1)
        topic=topic_sample.tolist().index(1)
        topic_words[topic].append(word)
        documents[i,j,:]=[word,topic]
the_one_true_document=''
for word in documents[0,:,0]:
    the_one_true_document+=(letters[int(word)])+' '
print the_one_true_document

true_data=np.zeros((20,3))
for word in letters:
    word_index=letters.index(word)
    for i in range(3):
        true_data[word_index,i]=topic_words[i].count(word_index)/len(topic_words[i])
    print word,topic_words[0].count(word_index)/len(topic_words[0])

# part II
bag_o_words=np.zeros((200,20))
for i in range(documents.shape[0]):
    for j in range(documents.shape[1]):
        word_index=int(documents[i,j,0])
        bag_o_words[i,word_index]+=1

lda=LatentDirichletAllocation(n_components=3,doc_topic_prior=0.1,
        topic_word_prior=0.01,learning_method='online')
lda.fit(bag_o_words)
total=0
recovered_data=np.zeros((20,3))
for topic_idx, topic in enumerate(lda.components_):
    recovered_data[:,topic_idx]=topic/sum(topic)
    # part III
    for word in topic:
        P=word/sum(topic)
        inner_product=P*np.log(P)
        total+=inner_product
for i in range(3):
    print cosine_similarity([true_data[:,i]],[recovered_data[:,i]])
entropy=-total/3
print entropy

# extra street cred
data=open('medical.txt','rb').read()
data=data.split('.W')
clean_data_prototype=[]
for thing in data:
    clean_data_prototype.extend(thing.split('.A'))
clean_data=[]
for thing in clean_data_prototype:
    if '.M' not in thing:
        clean_data.append(thing)

#  print len(clean_data)

vector=CountVectorizer(max_df=0.95,min_df=5,stop_words='english')
fit=vector.fit_transform(clean_data[:1000])
list_o_words=vector.get_feature_names()

lda=LatentDirichletAllocation(n_components=15,doc_topic_prior=0.1,
topic_word_prior=0.001,learning_method='online')

lda.fit(fit)
print_top_words(lda,list_o_words,15)
