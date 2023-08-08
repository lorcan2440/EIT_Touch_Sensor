import logging
import pandas as pd
import numpy as np
import openpyxl


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


def merge_excel_files(filenames: list[str], output_filename: str):
    sheet_names = set.intersection(*[set(pd.ExcelFile(filename).sheet_names) for filename in filenames])
    # create these sheets in the output file
    workbook = openpyxl.Workbook()
    workbook.save(output_filename)
    for sheet_name in sheet_names:
        workbook.create_sheet(title=sheet_name)
    # merge the sheets
    for filename in filenames:
        with pd.ExcelWriter(output_filename, engine='openpyxl', mode='a', if_sheet_exists='overlay') as writer:
            for sheet_name in sheet_names:
                df = pd.read_excel(filename, sheet_name=sheet_name)
                df.to_excel(writer, sheet_name=sheet_name, index=False)
    # remove the default sheet if it exists
    workbook.remove(workbook.get_sheet_by_name('Sheet'))
    # save
    workbook.save(output_filename)


if __name__ == '__main__':
    merge_excel_files(
        ['output/EIT_Data_Gelatin_4_dof_Set_1.xlsx', 'output/EIT_Data_Gelatin_4_dof_Set_2.xlsx'],
        'output/EIT_Data_Gelatin_4_dof.xlsx')