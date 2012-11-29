TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
DB += acqiris_board.db
DB += acqiris_board.substitutions
DB += acqiris_channel.template
DB += acqiris_channel_extras.template
DB += acqiris_module.template
DB += acqiris_module_extras.template
DB += acqiris_trigger.template

# Create and install archive files
#ARCHIVE += acqiris.tpl-arch

# Create and install autosave files
REQ += acqiris.tpl-req


#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_TEMPLATE = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


