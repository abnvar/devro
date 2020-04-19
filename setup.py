import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
     name='devro',  
     version='0.0.1',
     author="Abhinav Arora",
     author_email="abhiar@iitk.ac.in",
     description="A development package for robotics software",
     long_description=long_description,
     long_description_content_type="text/markdown",
     url="https://github.com/abnvar/devro",
     packages=setuptools.find_packages(),
     classifiers=[
         "Programming Language :: Python :: 3",
         "License :: OSI Approved :: MIT License",
         "Operating System :: OS Independent",
     ],
 )
