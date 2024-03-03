from setuptools import setup, find_packages

setup(
    name='parallel_robot',
    version='0.1.0',
    author='Marti Salcedo Bosch',
    author_email='martisalcedo7@hotmail.com',
    packages=find_packages(),
    description='A simulation package for parallel robots.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    license='MIT',
    keywords='robot simulation',
    # url='https://github.com/yourusername/robot_simulator',  # Optional
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Libraries',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
    python_requires='>=3.7',
)