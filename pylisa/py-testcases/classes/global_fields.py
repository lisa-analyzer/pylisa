class Config:
    env_file = ".env"
    env_file_encoding = 'utf-8'
    case_sensitive = True
w = Config.case_sensitive
x = Config.env_file
y = Config.env_file_encoding
Config.case_sensitive = False
z = Config.env_file_encoding
print(w)
print(x)
print(y)
print(z)
config = Config()
a = config.case_sensitive
b = config.env_file
c = config.env_file_encoding
d = config.case_sensitive

print(a)
print(b)
print(c)
print(d)

config.case_sensitive = True
e = config.case_sensitive
f = Config.case_sensitive
print(e)
print(f)