// file_parameter.h
//
// Very simple XML-parameter from File - Reader
// Version 0.1 enclosing tags are ignored
//
// use simple XML-code in the file like:
//
// <param1 val1="7" val2="4.3" ... />
// <param2 val1=""  ...	/>
//
// Get the values like this:
//
// BOOL get_file_parameter(CString filename, CString param, CString val, double* retvalue);
// BOOL get_file_parameter(CString filename, CString param, CString val, );

#include "stdio.h"
#include <stdlib.h>
#include <string>


double get_parameter(std::string filename, std::string tag2search,  long tagnum, std::string attr2search, double std_value);
bool get_file_parameter(std::string filename, std::string tag2search, long tagnum, std::string attr2search, double* retvalue);
bool get_file_parameter(std::string filename, std::string tag2search, long tagnum, std::string attr2search, std::string* p_retvalue);


// Return the value from file or the given standard value if not possible
double get_parameter(std::string filename, std::string tag2search,  long tagnum, std::string attr2search, double std_value) {
	double value_readed, value2return;
	value_readed = value2return = 0L;
	if (get_file_parameter(filename, tag2search, tagnum, attr2search, &value_readed)) {
		printf("File: %s \tTag: %s \tAttr: %s \tValue %lf readed\n",filename.c_str(), tag2search.c_str(), attr2search.c_str(), value_readed);
		value2return = value_readed; 
	} else {
		printf("not found: %s \tTag: %s \tAttr: %s \tValue %lf readed\n",filename.c_str(), tag2search.c_str(), attr2search.c_str(), value_readed);
		value2return = std_value;
	}
	return value2return;
}

// Read the value from file an convert it to double given in p_retvalue
bool get_file_parameter(std::string filename, std::string tag2search, long tagnum, std::string attr2search, double* p_retvalue) {
	// Convert the value to double
	bool value_found;
	std::string attr_value;
	value_found = get_file_parameter(filename, tag2search, tagnum, attr2search, &attr_value);
	if (value_found) {
		// Convert the value to double
		*p_retvalue = atof(attr_value.c_str());
	}
	return value_found;
}

// Read the value from file an write it to string p_retvalue
bool get_file_parameter(std::string filename, std::string tag2search, long tagnum, std::string attr2search, std::string* p_retvalue) {
	// Open the File
	FILE* fd_file;
	fd_file = fopen(filename.c_str(),"r");
	if (fd_file == NULL) {
		printf("File not found\n");
		return false;
	}
	// clear return value
	bool value_found = false;
	*p_retvalue = "";
	// Data
	char ch;
	std::string tag_name, final_tag_name, attr_name, attr_value;
	long tag_counter = 0; 			// Counter for the number of tags with name tag2search
	// Statemachine
	enum XML_Status {	WAIT_TAG, COLLECT_TAG_NAME, COLLECT_TAG_FIRST_CHAR, COLLECT_COMMENT, COLLECT_FINAL_TAG_NAME, COLLECT_TAG_IM_CLOSED, \
				WAIT_ATTR, COLLECT_ATTR_NAME, WAIT_ATTR_VALUE, COLLECT_ATTR_VALUE, TAG_END, FINAL_TAG_END};
	XML_Status stat = WAIT_TAG;
	// Start reading	
	while (fread(&ch, 1, 1, fd_file)) {
		// Start
		// printf("Status %d Char%c \n", stat, ch);
		switch (stat) {
			case WAIT_TAG:
				// Waiting for leading <
				if (ch == '<') stat = COLLECT_TAG_FIRST_CHAR;
				break;
			case COLLECT_TAG_FIRST_CHAR:
				// Collect the first char after '<' 
				if ((ch == '/')||(ch == '!')) {
					if ((ch == '/')) {
						// Final Tag
						stat = COLLECT_FINAL_TAG_NAME;
					}
					if ((ch == '!')) {
						// Comment
						stat = COLLECT_COMMENT;
					}
				} else {
					// Normal ..
					tag_name += ch;
					stat = COLLECT_TAG_NAME;
				}
				break;
			case COLLECT_TAG_NAME:
				// Collect the tag name until whitespace
				if ((ch != ' ')&&(ch != '>')&&(ch != '/')) {
					// Collect the non ws chars
					tag_name += ch;
				} else {
					// Counter
					if (tag_name == tag2search) {
						tag_counter++;
					}
					// attributes?
					if (ch == '>') stat = TAG_END; else {
						// printf("Opening Tag is %s \n", tag_name.c_str());
						stat = WAIT_ATTR; 
					}
					// immediatly closing?
					if (ch == '/') stat = COLLECT_TAG_IM_CLOSED;
				}
				break;
			case COLLECT_TAG_IM_CLOSED:
				// Tag is immediatly closing (End is />)
				if (ch == '>') {
					// Copy the name ..
					final_tag_name = tag_name;
					stat = FINAL_TAG_END;
				} else printf("Error \n");
				break;
			case COLLECT_FINAL_TAG_NAME:
				// Collect the final tag name 
				if (ch != '>') {
					// Collect the non ws chars
					final_tag_name += ch;
				} else stat = FINAL_TAG_END;
				break;
			case WAIT_ATTR:
				// Collect the tag name until ' ' or '>' or '/'
				if ((ch != ' ')&&(ch != '>')&&(ch != '/')) {
					attr_name += ch;
					stat = COLLECT_ATTR_NAME;
				}
				if ((ch == '>')) {
					// Tag has no more attributes!
					stat =  TAG_END;
				}
				if (ch == '/') {
					// Tag is immediatly closing
					stat =  COLLECT_TAG_IM_CLOSED;
				}
				break;
			case COLLECT_ATTR_NAME:
				// Collect the attribute name until the '=' sign
				if (ch != '=')  {
					attr_name += ch;
				} else stat = WAIT_ATTR_VALUE;
				break;
			case WAIT_ATTR_VALUE:
				// Wait until the '\"' sign
				if (ch == '\"') stat =  COLLECT_ATTR_VALUE;
				break;
			case COLLECT_ATTR_VALUE:
				// Collect until the second '\"' sign
				if (ch != '\"')  {
					attr_value += ch;
				} else {
					// An attribute is complete, do something with this information
					// printf("<#%s# ", tag_name.c_str());
					// printf("#%s#=\"", attr_name.c_str());
					// printf("#%s#\">\n", attr_value.c_str());
					// Match?
					if ((tag_name == tag2search) && (tag_counter == tagnum) && (attr_name == attr2search)) {
						// printf("Value found in file, num: %ld, Attr: %s \n", tagnum, attr2search.c_str());
						value_found = true;
						*p_retvalue = attr_value;
					}
					// Clear variables for the next round
					attr_name = "";
					attr_value = "";
					stat = WAIT_ATTR;
				}
				break;
			case COLLECT_COMMENT:
				// Wait for end of comment
				if (ch != '>') stat =  WAIT_TAG;
				break;
			case TAG_END:
				// Do something?
				// printf("Opening Tag was %s \n", tag_name.c_str());
				tag_name = ""; 
				attr_name = "";
				attr_value = "";
				stat = WAIT_TAG;
				break;
			case FINAL_TAG_END:
				// Do something?
				// printf("Closing Tag was %s \n", final_tag_name.c_str());
				final_tag_name = "";
				tag_name = ""; 
				attr_name = "";
				attr_value = "";
				stat = WAIT_TAG;
				break;
		}
	}
	// Done
	fflush(stdout);
	fclose(fd_file);
	return value_found;
}



