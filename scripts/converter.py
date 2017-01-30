#!/usr/bin/python
import sys
import getopt
import os.path
from converter_tri_to_stl import *
from converter_stl_to_tri import *

class ConverterInterface:

  charLineBreak = '-'

  def NotYetImplemented(self):
    print "ERROR: No known convert from "+self.input_file_extension+" to "+self.output_file_extension
    sys.exit(1)

  def printUsage(self):
    print
    print "Usage: converter.py -i <input_filename> -e <output_file_extension>"
    print
    print " -h              : show usage"
    print " -e --extension  : output file extension"
    print
    sys.exit(0)

  def __init__(self, args):
    self.input_file_name = ''
    self.input_file_extension = ''
    self.output_file_name = ''
    self.output_file_extension = ''

    N=len(args)
    try:
      opts, args = getopt.getopt(args[1:],"hi:e:")
    except getopt.GetoptError:
      self.printUsage()
      sys.exit(0)

    for opt, arg in opts:
      if opt == '-h':
        self.printUsage()
      elif opt in ("-i", "--input_file_name"):
        self.input_file_name = arg
      elif opt in ("-e", "--extension"):
        self.output_file_extension = arg

    if self.input_file_name == '':
      self.printUsage()
    ## obtain input file extension
    self.input_file_name = os.path.abspath(self.input_file_name)
    basename = os.path.basename(self.input_file_name)
    basename = os.path.splitext(basename)
    self.input_file_extension = basename[-1][1:]
    basename = basename[0]

    ## convert to output file 
    pathname = os.path.dirname(os.path.realpath(__file__))+'/../data/'
    self.output_file_name = os.path.abspath(pathname)+'/'+basename+"."+self.output_file_extension

    print self.charLineBreak*80
    print "AnyConverter"
    print self.charLineBreak*80
    print "Input filename       : "+self.input_file_name
    print "Input file extension : "+self.input_file_extension
    print "Output filename      : "+self.output_file_name
    print "Output file extension: "+self.output_file_extension
    print self.charLineBreak*80

    if self.input_file_extension == 'tri':
      if self.output_file_extension == 'stl':
        ConverterTriToStl(self.input_file_name, self.output_file_name)
      else:
        NotYetImplemented()

    if self.input_file_extension == 'stl':
      if self.output_file_extension == 'tri':
        ConverterStlToTri(self.input_file_name, self.output_file_name)
      else:
        NotYetImplemented()
    print self.charLineBreak*80
    print "[SUCCESS]"
    print "Output written to "+self.output_file_name
    print self.charLineBreak*80

ConverterInterface(sys.argv)
