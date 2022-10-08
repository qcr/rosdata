#!/usr/bin/env python

###############
### MODULES ###
###############

import csv
import numpy as np

import spatialmath as sm

###############
### CLASSES ###
###############



class CSVROSDataRow():
    """A helper class to store a single row of data contained within a CSV file created by ROSData tools.
    """

    def __init__(self, data: list, fields: list) -> None:
        """Initialises a CSVROSDataRow object where the class attributes will those contained within the fields argument. For the pose data functions to be accesible the fields argument must contain the strings ['pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z']. Numeric data will be stored as floats, all other data will be stored as strings except for the string 'none' which will be stored as None.

        Args:
            data (list): the values for the provided fields
            fields (list): the names for each field provided within the data

        Raises:
            ValueError: if the length of the fields and data arguments are not equal.
        """
        
        # Check fields and data are same length
        if len(fields) != len(data):
            raise ValueError("The number of fields must be equal to size of the data")

        # Attempt to convert data to float
        for idx in range(len(data)):
            val = data[idx]
            try:
                # attempt to convert to float
                val = float(val)
            except ValueError:
                if val.lower() == 'none':
                    # if was the string none, then convert to None type
                    val = None
            
            # set val
            data[idx] = val

        # Set class variables (attributes)
        for idx, field in enumerate(fields):
            setattr(self, field, data[idx])


    def get_pose(self) -> sm.SE3:
        """Will retrieve the pose as a Spatial Maths SE3 object. The pose data must be stored under the headers ['pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z'].

        Returns:
            sm.SE3: the pose data as a SE object or None if the pose data does not exist.
        """

        data = self.get_pose_data()
        if data is None:
            return None
        
        # Conver to spatialmath.SE3 object
        se3 = sm.SE3(data[:3])
        se3.A[:3, :3] = sm.base.q2r(data[3:])
        if not isinstance(se3.A, np.ndarray):
            return None

        # return se3
        return se3

    def get_pose_data(self) -> list:
        """Gets the pose data as individual float values. The pose data must be stored under the headers ['pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z'].

        Returns:
            list: the pose data as a list of floats. Order will be ['pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z'].
        """

        data = [getattr(self, x) for x in ['pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z']]
        if None in data or len(data) != 7:
            return None
        return data




class CSVROSData():
    """A utility class to provide easy accessibility to a CSV file created by ROSData tools.
    """

    def __init__(self, csvfile: str) -> None:
        """Initialises a CSVROSData object where the class attributes, referred to as fields, will be the headers of the CSV file. For the pose data functions to be accesible the CSV file must contain the headers ['pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z']. Numeric data will be stored as floats, all other data will be stored as strings except for the string 'none' which will be stored as None.

        Args:
            csvfile (str): the path to the ROSData CSV file
        """

        # Class Variables
        self._data = []
        self._fields = []

        # Read CSV file
        with open(csvfile, newline='') as f:
            csvreader = csv.reader(f, delimiter=',')
            for idx, row in enumerate(csvreader):
                if idx == 0:
                    self._fields = row
                else:
                    self._data.append(CSVROSDataRow(row, self._fields))
            f.close()       


    def pose_data_exists(self) -> bool:
        """Returns true if the CSV ROSData file contained pose information.

        Returns:
            bool: true if pose information is available.
        """
        if self._data[0].get_pose_data():
            return True
        return False


    def get_pose(self, index : int) -> sm.SE3:
        """Gets the transform (spatialmath.SE3 object) for the given index.

        Args:
            index (int): the index for the desired transform

        Returns:
            spatialmath.SE3: returns a spatialmath.SE3 object with the given transform, or None if a transform does not exist for this index.
        """

        return self._data[index].get_pose()    


    def get_pose_data(self, index : int) -> list:
        """Gets the transform (spatialmath.SE3 object) for the given index.

        Args:
            index (int): the index for the desired transform

        Returns:
            spatialmath.SE3: returns a spatialmath.SE3 object with the given transform, or None if a transform does not exist for this index.
        """

        return self._data[index].get_pose_data()


    def field_exists(self, field : str) -> bool:
        """Checks to see if a field exists within this CSV data

        Args:
            field (str): the field

        Returns:
            bool: true if the field exists
        """

        if field in self._fields:
            return True
        return False

    
    def get_data(self, indices=None, fields=None):
        """Gets the entire data for a specific index, or the value for a field for a index, or
        gets the field for the entire data (e.g., all timestamps). 

        Examples:
            
            | # return data for index 0
            | data = csvrosdata_obj.get_data(0)   
            
            | # return all pos_x data
            | data = csvrosdata_obj.get_data('pos_x') 
            
            | # return all fields for multiple indices
            | data = csvrosdata_obj.get_data([0, 2])  

            | # return all data for a set of fields
            | data = csvrosdata_obj.get_data(['pos_x', 'pos_z'])  
            
            | # return multiple fields for a specified index or a set of indices
            | data = csvrosdata_obj.get_data(0, ['pos_x', 'pos_z']) 
            | data = csvrosdata_obj.get_data([0, 2], ['pos_x', 'pos_z'])  

        Args:
            indices (int, str, list): the index or indices to be retrieved
            fields (optional, int or str): the field or fields to be retrieved

        Raises:
            ValueError: if too many arguments are provided

        Returns:
            variable: either the data (list) for a given index, the value for a given index/field or the data (list) for a field across all indices.
            
        """

        # Argument check and conversion to correct format
        if indices is None:
            indices = []
        elif isinstance(indices, (np.integer, np.float, int, float)):
            indices = [int(indices)] # change to int and convert to list
        elif isinstance(indices, list) and all(isinstance(x, (np.integer, np.float, int, float)) for x in indices):
            indices = [int(x) for x in indices]
        else:
            raise ValueError("The indices argument must be a integer, float or list of integers or floats.")

        if fields is None:
            fields = []
        elif isinstance(fields, str):
            fields = [fields]
        elif isinstance(fields, list) and all(isinstance(x, str) for x in fields):
            pass # don't need to do anything
        else:
            raise ValueError("The fields argument must be a string, or list of strings.")

        if len(indices) != 0 and len(fields) != 0:
            retval = []
            for x in indices:
                if len(fields) == 1:
                    retval.append(getattr(self._data[x], fields[0]))
                else:
                    retval.append([getattr(self._data[x], y) for y in fields])
        elif len(indices) != 0:
            retval = [self._data[x] for x in indices]
        elif len(fields) != 0:
            retval = []
            for x in self._data:
                if len(fields) == 1:
                    retval.append(getattr(x, fields[0]))
                else:
                    retval.append([getattr(x, y) for y in fields])

        # only return the element if single item in list
        if len(retval) == 1:
            return retval[0]
        return retval
        


    def __getitem__(self, item):
        """Can be used as a shorthand for the get_data method. 
        
        Examples:

            | # equivalent to csvrosdata_obj.get_data(indices=0)
            | csvrosdata_obj[0] 

            | # equivalent to csvrosdata_obj.get_data(fields='timestamp')
            | csvrosdata_obj['timestamp'] 

            | # equivalent to csvrosdata_obj.get_data(indices=0, fields='timestamp')
            | csvrosdata_obj[0, 'timestamp'] 
        """

        if isinstance(item, tuple):
            return self.get_data(*item)

        elif isinstance(item, (int, np.integer, float, np.float)):
            return self.get_data(indices=item)
        elif isinstance(item, list) and all(isinstance(x, (np.integer, np.float, int, float)) for x in item):
            return self.get_data(indices=item)

        elif isinstance(item, str):
            return self.get_data(fields=item)
        elif isinstance(item, list) and all(isinstance(x, str) for x in item):
            return self.get_data(fields=item)

        else:
            raise ValueError("Unknown argument type")


    def __len__(self) -> int:
        """returns the length of the data

        Returns:
            int: the length of the data
        """

        return len(self._data)


########################
### PUBLIC FUNCTIONS ###
########################



#########################
### PRIVATE FUNCTIONS ###
#########################