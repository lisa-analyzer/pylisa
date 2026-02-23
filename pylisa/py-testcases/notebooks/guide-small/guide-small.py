import pandas as pd
df = pd.read_csv('temporal.csv')
df.head(10) 
df['Mes'] = pd.to_datetime(df['Mes'])
from geopandas.tools import geocode
df2 = pd.read_csv('mapa.csv')
df2.dropna(axis=0, inplace=True)
df2['geometry'] = geocode(df2['País'], provider='nominatim')['geometry']
df2['Latitude'] = df2['geometry'].apply(lambda l: l.y)
df2['Longitude'] = df2['geometry'].apply(lambda l: l.x)

df2.loc[df2["col2"] < 5, ['cool-col']] = 100

# concat along rows - combine the rows
df3 = pd.concat([df, df2])

# concat along columns - combine the column
df4 = df.join(df2)

# concat along rows - with the same dataframe
df5 = pd.concat([df['bar'], df['foo']])
