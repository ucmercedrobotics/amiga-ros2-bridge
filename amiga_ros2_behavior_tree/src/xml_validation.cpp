#include "amiga_ros2_behavior_tree/xml_validation.hpp"

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

namespace xml_validation
{

namespace
{
struct XmlInit
{
    XmlInit() { xmlInitParser(); }
    ~XmlInit() { xmlCleanupParser(); }
};
}

bool validate(const std::string &xml, const std::string &schema_path, std::string &error_out)
{
    static XmlInit init_guard;

    xmlDocPtr doc = xmlReadMemory(xml.c_str(), static_cast<int>(xml.size()), nullptr, nullptr, XML_PARSE_NONET | XML_PARSE_NOENT | XML_PARSE_NOBLANKS);
    if (!doc)
    {
        error_out = "Failed to parse XML string";
        return false;
    }

    xmlSchemaParserCtxtPtr pctxt = xmlSchemaNewParserCtxt(schema_path.c_str());
    if (!pctxt)
    {
        xmlFreeDoc(doc);
        error_out = "Failed to create schema parser context";
        return false;
    }

    xmlSchemaPtr schema = xmlSchemaParse(pctxt);
    xmlSchemaFreeParserCtxt(pctxt);
    if (!schema)
    {
        xmlFreeDoc(doc);
        error_out = "Failed to parse schema";
        return false;
    }

    xmlSchemaValidCtxtPtr vctxt = xmlSchemaNewValidCtxt(schema);
    if (!vctxt)
    {
        xmlSchemaFree(schema);
        xmlFreeDoc(doc);
        error_out = "Failed to create validation context";
        return false;
    }

    int ret = xmlSchemaValidateDoc(vctxt, doc);

    xmlSchemaFreeValidCtxt(vctxt);
    xmlSchemaFree(schema);
    xmlFreeDoc(doc);

    if (ret == 0)
    {
        return true;
    }
    else if (ret > 0)
    {
        error_out = "XML does not conform to the schema";
        return false;
    }
    else
    {
        error_out = "Internal validation error";
        return false;
    }
}

} // namespace xml_validation
