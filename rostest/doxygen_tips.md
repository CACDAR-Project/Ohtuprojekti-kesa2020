# Documentation Tips

## How to document the project

This is a brief instruction for making doxygen documents.

Doxygen finds comments written in the following style (leave out the \\):

    ## \@package pyexample
    #  Documentation for this module.
    #
    #  More details.
    ## Documentation for a function.
    #
    #  More details.
    def func():
       pass
    ## Documentation for a class.
    #
    #  More details.
    class PyClass:
     
       ## The constructor.
       def __init__(self):
           self._memVar = 0;
     
       ## Documentation for a method.
       #  @param self The object pointer.
       def PyMethod(self):
           pass
      
       ## A class variable.
       classVar = 0;
       ## \@var _memVar
       #  a member variable


This example and further information can be found at:
https://www.doxygen.nl/manual/docblocks.html#pythonblocks


### Tags

You can use tags in Python files, and Doxygen will list them under [Related Pages](pages.html).
   
\## \@bug Lets fix this  

Some useful tags:  
+    @bug  
+    @todo  
+    @warning
+    @test   

You can run doxygen in root with    

    doxygen    
   
The documentation files can be found in the documentation-folder.    

[Project in github](https://github.com/Konenako/Ohtuprojekti-kesa2020)