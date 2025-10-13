#pragma once

#include <string>

namespace xml_validation
{

// Validate an XML string against an XSD schema file.
// Returns true if valid; false otherwise and fills error_out with details.
bool validate(const std::string &xml, const std::string &schema_path, std::string &error_out);

}
