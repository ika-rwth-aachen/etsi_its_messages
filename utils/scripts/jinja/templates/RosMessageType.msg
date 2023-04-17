# ------------------------------------------------------------------------------
{% for line in asn1_definition.split('\n') -%}
# {{ line }}
{% endfor -%}
# ------------------------------------------------------------------------------

{% for member in members -%}
{{ member.type }} {{ member.name }}
{% for const in member.constants -%}
{{ const.type }} {{ const.name }} = {{ const.value }}
{% endfor %}
{% endfor %}