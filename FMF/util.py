#HHrecode v1.0 25/12/2021

import pandas as pd
import numpy as np 
import matplotlib.pyplot as plt



# Tra ve mot dictionary, key = id, value = [rating trung binh, rating max, rating min]
# Dau vao la 2 column ids va rating
def getScoreBroad(broad_ids,movie_rating):
  cnt = {}
  sum = {}
  for i in range(len(movie_rating)):
    if (np.isnan(movie_rating[i])):
      continue
    ids = str(broad_ids[i])
    ids = ids.replace('\'','').replace('[','').replace(']','').split(sep = ', ')

    for id in ids:
      if (id==''):
        continue
      if (cnt.get(id)==None): # khong co trong dict
        cnt[id] = 1
        sum[id] = [float(movie_rating[i])]*3
      else:
        cnt[id] += 1 # dem so luong
        sum[id][0] += movie_rating[i] # tong
        sum[id][1] = max(sum[id][1],movie_rating[i]) # max
        sum[id][2] = min(sum[id][2],movie_rating[2]) # min

  for id in cnt:
    sum[id][0] = sum[id][0]/cnt[id] # lay trung binh

  return sum


# Tra ve mot tuple [score trung binh, socre max cua lead, score min cua lead]
# Lead la nguoi co score trung binh lon nhat
# Dau vao la danh sach ids va dictionary
def getScore(ids,scoreBroad):
  re = 0
  lead_score = -1
  lead_id = 0
  cnt = 0;
  ids = str(ids)
  ids = ids.replace('\'','').replace('[','').replace(']','').split(sep = ', ')
  for id in ids:
    if (scoreBroad.get(id)!=None) and (int(id)!=0):
      cnt+=1
      re += scoreBroad[id][0]
      if (scoreBroad[id][0]>lead_score):
        lead_score = scoreBroad[id][0]
        lead_id = id


  if (cnt==0):
    return [np.nan]*3

  ret = [re/cnt, scoreBroad[lead_id][1], scoreBroad[lead_id][2]]
  return ret


# Chuyen string thanh list
def apply_genre(s):
    s = str(s)
    s = s.replace('\'','').replace('[','').replace(']','').split(sep = ', ')
    return s


# Tra ve danh sach gom tat ca cac genre
# Dau vao la danh sach genre cua tat ca cac phim
def getGenres(mgenre):
  mlist = []
  for i in range(len(mgenre)):
    mlist.extend(mgenre[i])
  genres = list(set(mlist))
  return genres


# Tra ve ones_hot vector tuong ung voi danh sach genre
# Dau vao la danh sach genre can ones_hot va danh sach tat ca genre
def getBow(genreList,genres):
  genre_bow = np.zeros(len(genres)+1)
  cnt = 0

  for i in range(len(genres)):
    if genres[i] in genreList:
      genre_bow[i] = 1
      cnt+=1

  if (cnt<len(genreList)): 
    genre_bow[-1] = 1
    
  return genre_bow
