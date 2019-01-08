#!/usr/bin/python
# -*- coding: UTF-8 -*-

import sys
import getopt
import uuid
import logging
import re
import os


def setup_logger(name):
    """配置 logger
    """
    root_logger = logging.getLogger(name)

    log_level = logging.DEBUG
    log_format = '%(asctime)-15s %(levelname)-8s - %(name)-5s:%(lineno)-3s %(message)s'

    formatter = logging.Formatter(log_format)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(formatter)

    root_logger.setLevel(level=log_level)
    root_logger.addHandler(handler)


def Generate(template_filename, target_filename, replace_mapping):
    """从模板文件中生成dsc文件

    Args:
        template_filename: string, 模板文件名, e.g. 'forage.dsc.template'
        target_filename: string, 生成的目标文件名, e.g. 'forage.dsc'
        replace_mapping: 模板文件中需要替换的字符串以及目标字符串
            e.g. {'<forage.class>':'com.github.hy.simple.froage',
                    '<escape.class>':'com.github.hy.simple.escape'}
    """
    dirname = os.path.dirname(target_filename)
    if not os.path.exists(dirname):
        os.makedirs(dirname)

    template_file = open(template_filename, 'r')
    target_file = open(target_filename, 'w')

    rep = dict((re.escape(k), v) for k, v in replace_mapping.items())
    pattern = re.compile("|".join(rep.keys()))

    try:
        for line in template_file:
            newline = pattern.sub(lambda m: rep[re.escape(m.group(0))], line)
            target_file.write(newline)
    finally:
        template_file.close()
        target_file.close()


def main(argv):
    usage = 'run.py -f <forage_class> -e <escape_class> -t <template_file>'
    forage_class = ''
    escape_class = ''

    try:
        opts, args = getopt.getopt(
            argv, "hf:e:t:", ["forage=", "escape=", "template="])
    except getopt.GetoptError:
        print(usage)
        sys.exit(2)

    if len(opts) != 3:
        print(usage)
        sys.exit()

    for opt, arg in opts:
        if opt == '-h':
            print(usage)
            sys.exit()
        elif opt in ("-f", "--forage"):
            forage_class = arg
        elif opt in ("-e", "--escape"):
            escape_class = arg
        elif opt in ("-t", "--template"):
            template_filename = arg

    logger.debug('forage_class：%s', forage_class)
    logger.debug('escape_class：%s', escape_class)
    logger.debug('template_filename: %s', template_filename)
    replace_mapping = {'<forage.class>': forage_class,
                       '<escape.class>': escape_class}

    dsc_filename = str(uuid.uuid1()) + '.dsc'
    filepath = os.path.split(os.path.realpath(__file__))[0]
    logger.debug('filepath: %s', filepath)
    dsc_filename = os.path.join(filepath, "bin", "dsc_tmp", dsc_filename)
    logger.debug('dsc_filename: %s', dsc_filename)
    Generate(template_filename, dsc_filename, replace_mapping)

    src_path = os.path.join(filepath, "src")
    dst_path = os.path.join(filepath, "bin")
    if not os.path.exists(dst_path):
        os.makedirs(dst_path)

    TBSimPath = os.path.join(os.path.split(
        os.path.split(filepath)[0])[0], "src")

    logger.debug('src_path: %s', src_path)
    logger.debug('dst_path: %s', dst_path)
    logger.debug('TBSimPath: %s', TBSimPath)
    classpath = "{0}{2}{1}".format(dst_path, TBSimPath, os.pathsep)
    logger.debug("classpath: %s", classpath)

    compile_cmd = 'find . -name "*.java" | xargs javac -encoding utf-8 -sourcepath {0} -cp {1} -d {2}'.format(
        src_path, TBSimPath, dst_path)
    logger.info("Compile: %s", compile_cmd)
    os.system(compile_cmd)

    cmd = 'java -classpath {0} TBSim.TBSim {1}'.format(classpath, dsc_filename)
    logger.info("Run: %s", cmd)
    os.system(cmd)


setup_logger('')
logger = logging.getLogger(__name__)
if __name__ == "__main__":
    main(sys.argv[1:])
