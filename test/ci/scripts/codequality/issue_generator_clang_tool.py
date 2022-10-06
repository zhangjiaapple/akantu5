#!/usr/bin/env python3

from . import print_debug, print_info
from .issue_generator import IssueGenerator
import os
import re
import copy
import json
import subprocess


class ClangToolIssueGenerator(IssueGenerator):
    """issue generator for clang tidy"""

    # 7-bit C1 ANSI sequences
    ANSI_ESCAPE = re.compile(r'''
        \x1B  # ESC
        (?:   # 7-bit C1 Fe (except CSI)
            [@-Z\\-_]
        |     # or [ for CSI, followed by a control sequence
            \[
            [0-?]*  # Parameter bytes
            [ -/]*  # Intermediate bytes
            [@-~]   # Final byte
        )
    ''', re.VERBOSE)

    def __init__(self, tool, **kwargs):
        self._tool = tool
        opts = copy.copy(kwargs)
        super().__init__(**kwargs)

        compiledb_path = opts.pop('compiledb_path')
        arguments = opts.pop('arguments', None)
        clang_tool = opts.pop('clang_tool_executable', tool)

        self._command = [clang_tool]

        if 'need_compiledb' in opts and opts['need_compiledb']:
            self._command.extend(['-p', compiledb_path])
        if arguments is not None:
            self._command.extend(arguments)

        if len(self._files) == 0 and compiledb_path:
            self._get_files_from_compile_db(compiledb_path)

    def _get_files_from_compile_db(self, compiledb_path):
        file_list = []
        with open(os.path.join(
                compiledb_path,
                'compile_commands.json'), 'r') as compiledb_fh:
            compiledb = json.load(compiledb_fh)
            for entry in compiledb:
                file_list.append(entry['file'])
        self._files = file_list
        self._filter_file_list()

    def _run_command(self, command):
        print_info(f'''[{self._tool}] command: {' '.join(command)}''')
        popen = subprocess.Popen(command,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL,
                                 universal_newlines=True)

        for stdout_line in iter(popen.stdout.readline, ""):
            clean_line = self.ANSI_ESCAPE.sub('', stdout_line)
            yield clean_line

        popen.stdout.close()

        return_code = popen.wait()
        if return_code:
            print_debug(
                f"[{self._tool}] {command} ReturnCode {return_code}")
