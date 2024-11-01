# This file contains compilation instructions of the application and FreeRTOS.
# THIS IS NOT A STANDALONE MAKE FILE. 
# ==========================================
# Possible improvements:
# * TODO: Include Eigen 
# ==========================================
# Made by Jorge Pérez 4 Owltech <3


C_INCLUDES += \
	-IMiddlewares/owlware/inc

# Note:
# By default, this will include everyting in the folder.
# If you want to build just certain classes, then add the 
# path directly. EJ:
# C_SOURCES += Middlewares/owlware/Templates/TemplateCFile.c

C_SOURCES += $(shell find Middlewares/owlware/src/*.c)