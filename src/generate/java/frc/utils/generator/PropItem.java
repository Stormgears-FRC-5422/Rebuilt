package frc.utils.generator;

public class PropItem {

    public enum PropertyType {
        STRING("string", "String", "getString"),
        INTEGER("int", "int", "getInt"),
        NUMBER("number", "double", "getNumber"),
        BOOLEAN("boolean", "Boolean", "getBoolean");

        private final String propertyTag;
        private final String javaType;
        private final String methodName;

        PropertyType(String propertyTag, String javaType, String methodName) {
            this.propertyTag = propertyTag;
            this.javaType = javaType;
            this.methodName = methodName;
        }

        public static PropertyType fromTag(String tag) {
            String normalizedTag = tag.toLowerCase();
            for (PropertyType type : values()) {
                if (type.propertyTag.equals(normalizedTag)) {
                    return type;
                }
            }
            throw new IllegalArgumentException("Unknown property type: " + tag);
        }

        public String getJavaType() {
            return javaType;
        }

        public String getMethodName() {
            return methodName;
        }
    }

    private final PropertyType propertyType;
    private final String key;
    private final String prefix;
    private final String defaultVal;
    private final boolean isOption;

    public PropItem(String typeAndTags, String key, String defaultVal, String prefix) {
        String[] tags = typeAndTags.split(",");
        this.propertyType = PropertyType.fromTag(tags[0].trim());
        this.isOption = tags.length > 1 && tags[1].trim().equalsIgnoreCase("option");
        this.key = key;
        this.defaultVal = defaultVal;
        this.prefix = prefix;
    }

    public boolean isOption() {
        return isOption;
    }

    public PropertyType getPropertyType() {
        return propertyType;
    }

    public String getKey() {
        return key;
    }

    public String getPrefix() {
        return prefix;
    }

    public String getDefaultVal() {
        return defaultVal;
    }

    private String quote(String s) {
        return "\"" + s + "\"";
    }

    @Override
    public String toString() {
        String methodCall = "StormProp." + propertyType.methodName;
        String defaultValue = propertyType == PropertyType.STRING ? quote(defaultVal) : defaultVal;

        return String.format("public static final %s %s = %s(%s, %s, %s);",
                propertyType.javaType,
                key,
                methodCall,
                quote(prefix),
                quote(key),
                defaultValue);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        PropItem other = (PropItem) o;
        return propertyType == other.propertyType
                && key.equals(other.key)
                && prefix.equals(other.prefix)
                && defaultVal.equals(other.defaultVal);
    }

    @Override
    public int hashCode() {
        int result = propertyType.hashCode();
        result = 31 * result + key.hashCode();
        result = 31 * result + prefix.hashCode();
        result = 31 * result + defaultVal.hashCode();
        return result;
    }
}
