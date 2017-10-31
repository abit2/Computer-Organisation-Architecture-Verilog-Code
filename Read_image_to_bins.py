from PIL import Image
import numpy as np
import glob
import os
import tensorflow as tf





FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string('dataset_dir', 'D:/Diangarti/CNN', """Path to the CASIA Tampered data directory.""")



def read_inputs():

  if not FLAGS.dataset_dir:
    raise ValueError('Please supply a data_dir')
  data_dir = os.path.join(FLAGS.dataset_dir, 'tampered/')
  filenames = os.path.join(data_dir, '*.tiff')
  i = 0
  out = np.array([])
  for img in glob.glob(filenames):
      i = i+1
      out1 = load_img2bin(img,i)
      out = np.array(list(out)+list(out1),np.uint8)
      if not tf.gfile.Exists(img):
          raise ValueError('Failed to find file: ' + img)
      else:
          print('File found')   
 
  out.tofile("train_batch_1.bin")
  
 

def load_img2bin(filenames,i):
    
    print(filenames)
    im = Image.open(filenames)
    im = (np.array(im))

    r = im[:,:,0].flatten()
    g = im[:,:,1].flatten()
    b = im[:,:,2].flatten()
    label = [1]

    out = np.array(list(label) + list(r) + list(g) + list(b),np.uint8)
    print('success in opening image')
    return out
        

def load_dataset():
    dest_directory = FLAGS.dataset_dir
    
    print(dest_directory)
    
    read_inputs()
    
    with open("train_batch_1.bin", "r") as g:
       a = np.fromfile(g, dtype=np.uint8)
       print(a.size)
       print(a)
       print(a.reshape(6,12289))

             
def main(argv=None):
    print('starting')
    load_dataset()
    
    
if __name__ == '__main__':
    tf.app.run()
    
            
            


