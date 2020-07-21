#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll, Ioan Sucan, Luis G. Torres

from sys import argv, exit
from os.path import basename, splitext, exists
import os
import sqlite3
import datetime
plottingEnabled=True
try:
    import matplotlib
    matplotlib.use('pdf')
    from matplotlib import __version__ as matplotlibversion
    from matplotlib.backends.backend_pdf import PdfPages
    import matplotlib.pyplot as plt
    import numpy as np
    from math import floor
except:
    print('Matplotlib or Numpy was not found; disabling plotting capabilities...')
    plottingEnabled=False
from optparse import OptionParser, OptionGroup

# Given a text line, split it into tokens (by space) and return the token
# at the desired index. Additionally, test that some expected tokens exist.
# Return None if they do not.
def readLogValue(filevar, desired_token_index, expected_tokens) :
    start_pos = filevar.tell()
    tokens = filevar.readline().split()
    for token_index in expected_tokens:
        if not tokens[token_index] == expected_tokens[token_index]:
            # undo the read, if we failed to parse.
            filevar.seek(start_pos)
            return None
    return tokens[desired_token_index]

def readOptionalLogValue(filevar, desired_token_index, expected_tokens = {}) :
    return readLogValue(filevar, desired_token_index, expected_tokens)

def readRequiredLogValue(name, filevar, desired_token_index, expected_tokens = {}) :
    result = readLogValue(filevar, desired_token_index, expected_tokens)
    if result == None:
        raise Exception("Unable to read " + name)
    return result

def ensurePrefix(line, prefix):
    if not line.startswith(prefix):
        raise Exception("Expected prefix " + prefix + " was not found")
    return line

def readOptionalMultilineValue(filevar):
    start_pos = filevar.tell()
    line = filevar.readline()
    if not line.startswith("<<<|"):
        filevar.seek(start_pos)
        return None
    value = ''
    line = filevar.readline()
    while not line.startswith('|>>>'):
        value = value + line
        line = filevar.readline()
        if line == None:
            raise Exception("Expected token |>>> missing")
    return value

def readRequiredMultilineValue(filevar):
    ensurePrefix(filevar.readline(), "<<<|")
    value = ''
    line = filevar.readline()
    while not line.startswith('|>>>'):
        value = value + line
        line = filevar.readline()
        if line == None:
            raise Exception("Expected token |>>> missing")
    return value


def readBenchmarkLog(dbname, filenames):
    """Parse benchmark log files and store the parsed data in a sqlite3 database."""

    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')

    # create all tables if they don't already exist
    c.executescript("""CREATE TABLE IF NOT EXISTS experiments
        (id INTEGER PRIMARY KEY AUTOINCREMENT, name VARCHAR(512),
        totaltime REAL, timelimit REAL, memorylimit REAL, runcount INTEGER,
        version VARCHAR(128), hostname VARCHAR(1024), cpuinfo TEXT,
        date DATETIME, seed INTEGER, setup TEXT);
        CREATE TABLE IF NOT EXISTS plannerConfigs
        (id INTEGER PRIMARY KEY AUTOINCREMENT,
        name VARCHAR(512) NOT NULL, settings TEXT);
        CREATE TABLE IF NOT EXISTS enums
        (name VARCHAR(512), value INTEGER, description TEXT,
        PRIMARY KEY (name, value));
        CREATE TABLE IF NOT EXISTS runs
        (id INTEGER PRIMARY KEY AUTOINCREMENT, experimentid INTEGER, plannerid INTEGER,
        FOREIGN KEY (experimentid) REFERENCES experiments(id) ON DELETE CASCADE,
        FOREIGN KEY (plannerid) REFERENCES plannerConfigs(id) ON DELETE CASCADE);
        CREATE TABLE IF NOT EXISTS progress
        (runid INTEGER, time REAL, PRIMARY KEY (runid, time),
        FOREIGN KEY (runid) REFERENCES runs(id) ON DELETE CASCADE)""")

    for filename in filenames:
        print('Processing ' + filename)
        logfile = open(filename,'r')
        start_pos = logfile.tell()
        libname = readOptionalLogValue(logfile, 0, {1 : "version"})
        if libname == None:
            libname = "OMPL"
        logfile.seek(start_pos)
        version = readOptionalLogValue(logfile, -1, {1 : "version"})
        if version == None:
            # set the version number to make Planner Arena happy
            version = "0.0.0"
        version = ' '.join([libname, version])
        expname = readRequiredLogValue("experiment name", logfile, -1, {0 : "Experiment"})

        # optional experiment properties
        nrexpprops = int(readOptionalLogValue(logfile, 0, {-2: "experiment", -1: "properties"}) or 0)
        expprops = {}
        for i in range(nrexpprops):
            entry = logfile.readline().strip().split('=')
            nameAndType = entry[0].split(' ')
            expprops[nameAndType[0]] = (entry[1], nameAndType[1])

        # adding columns to experiments table
        c.execute('PRAGMA table_info(experiments)')
        columnNames = [col[1] for col in c.fetchall()]
        for name in sorted(expprops.keys()):
            # only add column if it doesn't exist
            if name not in columnNames:
                c.execute('ALTER TABLE experiments ADD %s %s' % (name, expprops[name][1]))

        hostname = readRequiredLogValue("hostname", logfile, -1, {0 : "Running"})
        date = ' '.join(ensurePrefix(logfile.readline(), "Starting").split()[2:])
        expsetup = readRequiredMultilineValue(logfile)
        cpuinfo = readOptionalMultilineValue(logfile)
        rseed = int(readRequiredLogValue("random seed", logfile, 0, {-2 : "random", -1 : "seed"}))
        timelimit = float(readRequiredLogValue("time limit", logfile, 0, {-3 : "seconds", -2 : "per", -1 : "run"}))
        memorylimit = float(readRequiredLogValue("memory limit", logfile, 0, {-3 : "MB", -2 : "per", -1 : "run"}))
        nrrunsOrNone = readOptionalLogValue(logfile, 0, {-3 : "runs", -2 : "per", -1 : "planner"})
        nrruns = -1
        if nrrunsOrNone != None:
            nrruns = int(nrrunsOrNone)
        totaltime = float(readRequiredLogValue("total time", logfile, 0, {-3 : "collect", -2 : "the", -1 : "data"}))
        numEnums = 0
        numEnumsOrNone = readOptionalLogValue(logfile, 0, {-2 : "enum"})
        if numEnumsOrNone != None:
            numEnums = int(numEnumsOrNone)
        for i in range(numEnums):
            enum = logfile.readline()[:-1].split('|')
            c.execute('SELECT * FROM enums WHERE name IS "%s"' % enum[0])
            if c.fetchone() == None:
                for j in range(len(enum)-1):
                    c.execute('INSERT INTO enums VALUES (?,?,?)',
                        (enum[0],j,enum[j+1]))

        # Creating entry in experiments table
        experimentEntries = [None, expname, totaltime, timelimit, memorylimit, nrruns, version,
                             hostname, cpuinfo, date, rseed, expsetup]
        for name in sorted(expprops.keys()): # sort to ensure correct order
            experimentEntries.append(expprops[name][0])
        c.execute('INSERT INTO experiments VALUES (' + ','.join('?' for i in experimentEntries) + ')', experimentEntries)
        experimentId = c.lastrowid

        numPlanners = int(readRequiredLogValue("planner count", logfile, 0, {-1 : "planners"}))
        for i in range(numPlanners):
            plannerName = logfile.readline()[:-1]
            print('Parsing data for ' + plannerName)

            # read common data for planner
            numCommon = int(logfile.readline().split()[0])
            settings = ''
            for j in range(numCommon):
                settings = settings + logfile.readline() + ';'

            # find planner id
            c.execute('SELECT id FROM plannerConfigs WHERE (name=? AND settings=?)',
                (plannerName, settings,))
            p = c.fetchone()
            if p==None:
                c.execute('INSERT INTO plannerConfigs VALUES (?,?,?)',
                    (None, plannerName, settings,))
                plannerId = c.lastrowid
            else:
                plannerId = p[0]

            # get current column names
            c.execute('PRAGMA table_info(runs)')
            columnNames = [col[1] for col in c.fetchall()]

            # read properties and add columns as necessary
            numProperties = int(logfile.readline().split()[0])
            propertyNames = ['experimentid', 'plannerid']
            for j in range(numProperties):
                field = logfile.readline().split()
                propertyType = field[-1]
                propertyName = '_'.join(field[:-1])
                if propertyName not in columnNames:
                    c.execute('ALTER TABLE runs ADD %s %s' % (propertyName, propertyType))
                propertyNames.append(propertyName)
            # read measurements
            insertFmtStr = 'INSERT INTO runs (' + ','.join(propertyNames) + \
                ') VALUES (' + ','.join('?'*len(propertyNames)) + ')'
            numRuns = int(logfile.readline().split()[0])
            runIds = []
            for j in range(numRuns):
                values = tuple([experimentId, plannerId] + \
                    [None if len(x) == 0 or x == 'nan' or x == 'inf' else x
                    for x in logfile.readline().split('; ')[:-1]])

                c.execute(insertFmtStr, values)
                # extract primary key of each run row so we can reference them
                # in the planner progress data table if needed
                runIds.append(c.lastrowid)

            nextLine = logfile.readline().strip()

            # read planner progress data if it's supplied
            if nextLine != '.':
                # get current column names
                c.execute('PRAGMA table_info(progress)')
                columnNames = [col[1] for col in c.fetchall()]

                # read progress properties and add columns as necesary
                numProgressProperties = int(nextLine.split()[0])
                progressPropertyNames = ['runid']
                for i in range(numProgressProperties):
                    field = logfile.readline().split()
                    progressPropertyType = field[-1]
                    progressPropertyName = "_".join(field[:-1])
                    if progressPropertyName not in columnNames:
                        c.execute('ALTER TABLE progress ADD %s %s' %
                            (progressPropertyName, progressPropertyType))
                    progressPropertyNames.append(progressPropertyName)
                # read progress measurements
                insertFmtStr = 'INSERT INTO progress (' + \
                    ','.join(progressPropertyNames) + ') VALUES (' + \
                    ','.join('?'*len(progressPropertyNames)) + ')'
                numRuns = int(logfile.readline().split()[0])
                for j in range(numRuns):
                    dataSeries = logfile.readline().split(';')[:-1]
                    for dataSample in dataSeries:
                        values = tuple([runIds[j]] + \
                            [None if len(x) == 0 or x == 'nan' or x == 'inf' else x
                            for x in dataSample.split(',')[:-1]])
                        try:
                            c.execute(insertFmtStr, values)
                        except sqlite3.IntegrityError:
                            print('Ignoring duplicate progress data. Consider increasing ompl::tools::Benchmark::Request::timeBetweenUpdates.')
                            pass

                logfile.readline()
        logfile.close()
    conn.commit()
    c.close()

def plotAttribute(cur, planners, attribute, typename):
    """Create a plot for a particular attribute. It will include data for
    all planners that have data for this attribute."""
    labels = []
    measurements = []
    nanCounts = []
    if typename == 'ENUM':
        cur.execute('SELECT description FROM enums where name IS "%s"' % attribute)
        descriptions = [ t[0] for t in cur.fetchall() ]
        numValues = len(descriptions)
    for planner in planners:
        cur.execute('SELECT %s FROM runs WHERE plannerid = %s AND %s IS NOT NULL' \
            % (attribute, planner[0], attribute))
        measurement = [ t[0] for t in cur.fetchall() if t[0] != None ]
        if len(measurement) > 0:
            cur.execute('SELECT count(*) FROM runs WHERE plannerid = %s AND %s IS NULL' \
                % (planner[0], attribute))
            nanCounts.append(cur.fetchone()[0])
            labels.append(planner[1])
            if typename == 'ENUM':
                scale = 100. / len(measurement)
                measurements.append([measurement.count(i)*scale for i in range(numValues)])
            else:
                measurements.append(measurement)

    if len(measurements)==0:
        print('Skipping "%s": no available measurements' % attribute)
        return

    plannerLabelRotation=80
    plt.clf()
    ax = plt.gca()
    if typename == 'ENUM':
        width = .5
        measurements = np.transpose(np.vstack(measurements))
        colsum = np.sum(measurements, axis=1)
        rows = np.where(colsum != 0)[0]
        heights = np.zeros((1,measurements.shape[1]))
        ind = range(measurements.shape[1])
        legend_labels = []
        for i in rows:
            plt.bar(ind, measurements[i], width, bottom=heights[0],
                color=matplotlib.cm.hot(int(floor(i*256/numValues))),
                label=descriptions[i])
            heights = heights + measurements[i]
        xtickNames = plt.xticks([x+width/2. for x in ind], labels,
            rotation=plannerLabelRotation)
        ax.set_ylabel(attribute.replace('_',' ') + ' (%)')
        box = ax.get_position()
        ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])
        props = matplotlib.font_manager.FontProperties()
        props.set_size('small')
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), prop = props)
    elif typename == 'BOOLEAN':
        width = .5
        measurementsPercentage = [sum(m) * 100. / len(m) for m in measurements]
        ind = range(len(measurements))
        plt.bar(ind, measurementsPercentage, width)
        xtickNames = plt.xticks([x + width / 2. for x in ind], labels, rotation=plannerLabelRotation)
        ax.set_ylabel(attribute.replace('_',' ') + ' (%)')
    else:
        if int(matplotlibversion.split('.')[0])<1:
            plt.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5)
        else:
            plt.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5, bootstrap=1000)
        ax.set_ylabel(attribute.replace('_',' '))
        xtickNames = plt.setp(ax,xticklabels=labels)
        plt.setp(xtickNames, rotation=plannerLabelRotation)
    ax.set_xlabel('Motion planning algorithm')
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    if max(nanCounts)>0:
        maxy = max([max(y) for y in measurements])
        for i in range(len(labels)):
            x = i+width/2 if typename=='BOOLEAN' else i+1
            ax.text(x, .95*maxy, str(nanCounts[i]), horizontalalignment='center', size='small')
    plt.show()

def plotStatistics(dbname, fname):
    print("Generating plots...")
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute('SELECT id, name FROM plannerConfigs')
    planners = [(t[0],t[1].replace('geometric_','').replace('control_',''))
        for t in c.fetchall()]

    print(c.fetchall())
    c.execute('PRAGMA table_info(runs)')
    colInfo = c.fetchall()[3:]

    runcount = np.array(c.execute('SELECT runcount FROM experiments;').fetchall()).flatten()[0]
    timelimit = np.array(c.execute("""SELECT timelimit FROM experiments;""").fetchall()).flatten()[0]

    pp = PdfPages(fname)
    Dsolved = 0
    for col in reversed(colInfo):
      #if col[1] == 'solved' or col[1] == 'time':
      if col[1] == 'solved':
        Dsolved = col[3]
      if col[1] == 'time':
        if col[2] == 'BOOLEAN' or col[2] == 'ENUM' or \
          col[2] == 'INTEGER' or col[2] == 'REAL':
          plotAttribute(c, planners, col[1], col[2])
          ax = plt.gca()
          ax.set_ylabel('Time (s)')
          ax.set_xlabel('Motion Planning Algorithm')
          ax.set_ylim([0,timelimit+0.15*timelimit]);
          txt = "runcount=%3.0d"%runcount
          ax.text(0.85, 0.95, txt, horizontalalignment='center', verticalalignment='center', transform = ax.transAxes)
          txt = "timelimit=%3.0f"%timelimit+"s"
          ax.text(0.85, 0.9, txt, horizontalalignment='center', verticalalignment='center', transform = ax.transAxes)
          ax.axhline(timelimit,color='k',linestyle='--')
          plt.tight_layout()
          pp.savefig(plt.gcf())

    pp.close()

def saveAsMysql(dbname, mysqldump):
    # See http://stackoverflow.com/questions/1067060/perl-to-python
    import re
    print("Saving as MySQL dump file...")

    conn = sqlite3.connect(dbname)
    mysqldump = open(mysqldump,'w')

    # make sure all tables are dropped in an order that keepd foreign keys valid
    c = conn.cursor()
    c.execute("SELECT name FROM sqlite_master WHERE type='table'")
    table_names = [ str(t[0]) for t in c.fetchall() ]
    c.close()
    last = ['experiments', 'planner_configs']
    for table in table_names:
        if table.startswith("sqlite"):
            continue
        if not table in last:
            mysqldump.write("DROP TABLE IF EXISTS `%s`;\n" % table)
    for table in last:
        if table in table_names:
            mysqldump.write("DROP TABLE IF EXISTS `%s`;\n" % table)

    for line in conn.iterdump():
        process = False
        for nope in ('BEGIN TRANSACTION','COMMIT',
            'sqlite_sequence','CREATE UNIQUE INDEX', 'CREATE VIEW'):
            if nope in line: break
        else:
            process = True
        if not process: continue
        line = re.sub(r"[\n\r\t ]+", " ", line)
        m = re.search('CREATE TABLE ([a-zA-Z0-9_]*)(.*)', line)
        if m:
            name, sub = m.groups()
            sub = sub.replace('"','`')
            line = '''CREATE TABLE IF NOT EXISTS %(name)s%(sub)s'''
            line = line % dict(name=name, sub=sub)
            # make sure we use an engine that supports foreign keys
            line = line.rstrip("\n\t ;") + " ENGINE = InnoDB;\n"
        else:
            m = re.search('INSERT INTO "([a-zA-Z0-9_]*)"(.*)', line)
            if m:
                line = 'INSERT INTO %s%s\n' % m.groups()
                line = line.replace('"', r'\"')
                line = line.replace('"', "'")

        line = re.sub(r"([^'])'t'(.)", "\\1THIS_IS_TRUE\\2", line)
        line = line.replace('THIS_IS_TRUE', '1')
        line = re.sub(r"([^'])'f'(.)", "\\1THIS_IS_FALSE\\2", line)
        line = line.replace('THIS_IS_FALSE', '0')
        line = line.replace('AUTOINCREMENT', 'AUTO_INCREMENT')
        mysqldump.write(line)
    mysqldump.close()


if __name__ == "__main__":
    usage = """%prog [options] [<benchmark.log> ...]"""
    parser = OptionParser(usage)
    parser.add_option("-d", "--database", dest="dbname", default="benchmark.db",
        help="Filename of benchmark database [default: %default]")

    if plottingEnabled:
        parser.add_option("-p", "--plot", dest="plot", default=None,
            help="Create a PDF of plots")
    (options, args) = parser.parse_args()

    if len(args)>0:
      readBenchmarkLog(options.dbname, args)

    if options.plot:
      plotStatistics(options.dbname, options.plot)
