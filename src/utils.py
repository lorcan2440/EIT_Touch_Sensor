import logging
import pandas as pd
import numpy as np
import math
import openpyxl
import os
import datetime as dt
import subprocess

class ColouredLoggingFormatter(logging.Formatter):

    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = '%(levelname)s: %(message)s'

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

def gear_metres_to_servo(dist: float) -> int:
    '''
    Convert a desired separation between fingers to a servo setting for the claw.
    
    ### Arguments
    #### Required
    - `dist` (float): centre-to-centre separation distance of fingers, in metres.
    #### Optional
    
    ### Returns
    - `int`: angle to turn the motor, in degrees.
    '''
    dist_mm = dist * 1000
    return int(math.degrees((dist_mm - 15) / 45))


def send_notification_alarm(title: str, message: str, callback_filename: str,
        script_path: str, logger: logging.Logger):
    '''
    Sends a Windows desktop notification (alarm) with a clickable callback to open a file.
    Uses `exp_complete_notifier.ps1` [self._NOTIFIER_SCRIPT] PowerShell script.
        
    #### Arguments
        
    `title` (str): notification title text.
    `message` (str): notification body text.
    `callback_filename` (str): file path to open when clicked, relative to cwd.
    '''
    # get log file from logger
    log_file = logger.handlers[0].baseFilename

    with open(log_file, 'a') as f:
        try:
            command = f'powershell.exe -ExecutionPolicy Bypass -File {script_path} ' + \
                f'"{title}" "{message}" "{callback_filename}"'
            subprocess.check_call(command, shell=True, stderr=f, stdout=f)
        except subprocess.CalledProcessError as e:
            logger.warning(f'Failed to send notification. Error: {e.returncode} - {e}')


def seconds_to_hms(seconds: float) -> str:
        '''
        Utility to convert a number of seconds to a string in the format 'HHhMMmSSs'.
        Used to report the time taken by the EIT experiments.
        
        ### Arguments
        #### Required
        - `seconds` (float): number of seconds.
        #### Optional
        
        ### Returns
        - `str`: string in the format 'HHhMMmSSs'.
        '''
        result = dt.datetime.strptime(
            str(dt.timedelta(seconds=round(seconds))), '%H:%M:%S').strftime('%Hh%Mm%Ss')
        return result


def merge_excel_files(filenames: list[str], output_filename: str):
    '''
    Merges the sheets of multiple excel files into a single excel file.
    If the target excel file already exists, appends without overwriting.
    
    ### Arguments
    #### Required
    - `filenames` (list[str]): list of filenames of Excel files to be merged.
    - `output_filename` (str): filename of the Excel file to be outputted.
    #### Optional
    '''    
    # init workbook
    sheet_names = set.intersection(*[set(pd.ExcelFile(filename).sheet_names) for filename in filenames])
    if not os.path.exists(output_filename):
        workbook = openpyxl.Workbook()
        workbook.save(output_filename)
        for sheet_name in sheet_names:
            workbook.create_sheet(title=sheet_name)
    else:
        workbook = openpyxl.load_workbook(output_filename)
    
    # merge the sheets
    for filename in filenames:
        with pd.ExcelWriter(output_filename, engine='openpyxl', mode='a', if_sheet_exists='overlay') as writer:
            for sheet_name in sheet_names:
                sheet_exists = (sheet_name in writer.sheets)
                df = pd.read_excel(filename, sheet_name=sheet_name)
                start_i = writer.sheets[sheet_name].max_row if sheet_exists else 0
                df.to_excel(writer, sheet_name=sheet_name, index=False,
                        startrow=start_i, header=(not sheet_exists))
                print(f'Rows: {len(df)}, Start: {start_i}, Sheet: {sheet_name}, Source: {filename}, Dest: {output_filename}')


def reading_num_to_electrodes(
        index: int, as_str: bool = False) -> tuple[int, int, int, int, bool] | str:
    '''
    Converts a given index of the voltage output to a tuple of
    the electrode numbers being used to take the measurement.
    
    ### Arguments
    #### Required
    - `index` (int): index of the voltage output, between 0 and 1023 inclusive.
    #### Optional
    - `as_str` (bool, default = False): if True, show a readable string of the electrodes.
    
    ### Returns
    - `tuple[int, int, int, int, bool] | str`: (i_src, i_sink, v_pos, v_neg, is_zero).
    '''    
    assert 0 <= index <= 1023
    
    i_src = int(index // 32) + 1
    i_sink = (i_src + 1) % 32
    v_pos = index % 32 + 1
    v_neg = (v_pos + 1) % 32

    # the readings will be zero if (v_pos, v_neg) overlap with (src_pin, sink_pin)
    if v_pos == i_src or v_pos == i_sink or v_neg == i_src or v_neg == i_sink:
        is_zero = True
    else:
        is_zero = False

    if as_str:
        return f'I_in: {i_src}, I_out: {i_sink}, V+: {v_pos}, V-: {v_neg}, Is zeroes: {is_zero}'
    else:
        return i_src, i_sink, v_pos, v_neg, is_zero


def electrodes_to_reading_num(current_source: int, v_pos: int) -> int:
    '''
    Convert a desired electrode combination to the index of the voltage output.
    
    ### Arguments
    #### Required
    - `current_source` (int): electrode number to input current at. 1 to 32 inclusive.
    - `v_pos` (int): electrode number to take as positive voltage. 1 to 32 inclusive.
    #### Optional
    
    ### Returns
    - `int`: index of corresponding voltage output, between 0 and 1023 inclusive.
    '''
    assert 1 <= current_source <= 32
    assert 1 <= v_pos <= 32
    return (current_source - 1) * 32 + (v_pos - 1)
