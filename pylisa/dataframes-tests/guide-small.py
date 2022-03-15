import pandas as pd
df = pd.read_csv('temporal.csv')
df.head(10) 
df['Mes'] = pd.to_datetime(df['Mes'])
from geopandas.tools import geocode
df2 = pd.read_csv('mapa.csv')
df2.dropna(axis=0, inplace=True)
df2['geometry'] = geocode(df2['País'], provider='nominatim')['geometry'] #It may take a while because it downloads a lot of data.
# TODO handle lambda and then re-enable these
#df2['Latitude'] = df2['geometry'].apply(lambda l: l.y)
#df2['Longitude'] = df2['geometry'].apply(lambda l: l.x)
