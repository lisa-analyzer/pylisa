def read_csv():
	import pandas as pd
	df = pd.read_csv('source.csv', sep='\n', verbose=True)

def read_pickle():
	import pandas as pd
	df = pd.read_pickle('source.pickle')

def read_table():
	import pandas as pd
	df = pd.read_table('source.table', sep='\n', verbose=True)

def read_fwf():
	import pandas as pd
	df = pd.read_fwf('source.fwf', infer_nrows=True)

def read_clipboard():
	import pandas as pd
	df = pd.read_clipboard('\n')

def read_excel():
	import pandas as pd
	df = pd.read_excel('source.xlsx', nrows=100)

def read_json():
	import pandas as pd
	df = pd.read_json('source.json', encoding='UTF-8')

def read_html():
	import pandas as pd
	df = pd.read_html('source.html', encoding='UTF-8')

def read_xml():
	import pandas as pd
	df = pd.read_xml('source.xml', encoding='UTF-8')

def read_hdf():
	import pandas as pd
	df = pd.read_hdf('source.hdf', start='x', stop='y', where='z')

def read_feather():
	import pandas as pd
	df = pd.read_feather('source.feather')

def read_parquet():
	import pandas as pd
	df = pd.read_parquet('source.parquet', engine='best')

def read_orc():
	import pandas as pd
	df = pd.read_orc('source.orc')

def read_sas():
	import pandas as pd
	df = pd.read_sas('source.sas', encoding='UTF-8')

def read_spss():
	import pandas as pd
	df = pd.read_spss('source.spss', sep='\n', usecols=True)

def read_stata():
	import pandas as pd
	df = pd.read_stata('source.stata', convert_dates=True)

def read_sql():
	import pandas as pd
	# none for the connection
	df = pd.read_sql('source.sql', None)

def read_sql_query():
	import pandas as pd
	# none for the connection
	df = pd.read_sql_query('source.query', None)

def read_sql_table():
	import pandas as pd
	# none for the connection
	df = pd.read_sql_table('source.sqlt', None, table_name='name')
