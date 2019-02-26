import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from edward.models import Normal
import edward as ed

def neural_network(x, W_0, W_1, b_0, b_1):
    h = tf.tanh(tf.matmul(x, W_0) + b_0)
    h = tf.matmul(h, W_1) + b_1
    return tf.reshape(h, [-1])

x_train=np.linspace(-3,3,50)
y_train=np.cos(x_train)+np.random.normal(0,0.1,50)
x_train=x_train.astype(np.float32).reshape((50,1))
y_train=y_train.astype(np.float32).reshape((50,1))

W_0=Normal(loc=tf.zeros([1,2]),scale=tf.ones([1,2]))
W_1=Normal(loc=tf.zeros([2,1]),scale=tf.ones([2,1]))
b_0=Normal(loc=tf.zeros(2),scale=tf.ones(2))
b_1=Normal(loc=tf.zeros(1),scale=tf.ones(1))

x=x_train
y=Normal(loc=tf.matmul(tf.tanh(tf.matmul(x,W_0)+b_0),W_1)+b_1,scale=0.1)

D=1

qW_0=Normal(loc=tf.get_variable("qW_0/loc",[D,2]),
        scale=tf.nn.softplus(tf.get_variable("qW_0/scale",[D,2])))
qW_1=Normal(loc=tf.get_variable("qW_1/loc",[2,1]),
        scale=tf.nn.softplus(tf.get_variable("qW_1/scale",[2,1])))
qb_0=Normal(loc=tf.get_variable("qb_0/loc",[2]),
        scale=tf.nn.softplus(tf.get_variable("qb_0/scale",[2])))
qb_1=Normal(loc=tf.get_variable("qb_1/loc",[1]),
        scale=tf.nn.softplus(tf.get_variable("qb_1/scale",[1])))

inputs = np.linspace(-5, 5, num=400, dtype=np.float32)
x = tf.expand_dims(inputs, 1)
mus = tf.stack(
    [neural_network(x, qW_0.sample(), qW_1.sample(),
                    qb_0.sample(), qb_1.sample())
        for _ in range(10)])

interface=ed.KLqp({W_0:qW_0,b_0:qb_0,W_1:qW_1,b_1:qb_1},data={y:y_train})
interface.run(n_iter=1000)

outputs=mus.eval()

fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)
ax.set_title("Iteration: 0")
ax.plot(x_train, y_train, 'ks', alpha=0.5, label='(x, y)')
ax.plot(inputs, outputs[0].T, 'r', lw=2, alpha=0.5, label='draws')
ax.set_xlim([-5, 5])
ax.set_ylim([-2, 2])
ax.legend()
plt.show()
